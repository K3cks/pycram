from threading import Lock
from typing_extensions import Any

import actionlib

from .. import world_reasoning as btr
import numpy as np
import rospy

from ..multirobot import RobotManager
from ..process_module import ProcessModule, ProcessModuleManager
from ..external_interfaces.ik import request_ik
from ..utils import _apply_ik
from ..local_transformer import LocalTransformer
from ..designators.object_designator import ObjectDesignatorDescription
from ..designators.motion_designator import MoveMotion, LookingMotion, \
    DetectingMotion, MoveTCPMotion, MoveArmJointsMotion, WorldStateDetectingMotion, MoveJointsMotion, \
    MoveGripperMotion, OpeningMotion, ClosingMotion
from ..robot_description import RobotDescription
from ..datastructures.world import World
from ..world_concepts.world_object import Object
from ..datastructures.pose import Pose
from ..datastructures.enums import JointType, ObjectType, Arms, ExecutionType
from ..external_interfaces import giskard
from ..external_interfaces.robokudo import *

try:
    from tiago_controllers_msgs.msg import tiagoGripperCommandGoal, tiagoGripperCommandAction, tiago
except ImportError:
    pass


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arms of tiago and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = RobotManager.active_robot
    if arm == "right":
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("right", "park").items():
            robot.set_joint_position(joint, pose)
    if arm == "left":
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("left", "park").items():
            robot.set_joint_position(joint, pose)


class tiagoNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion):
        robot = RobotManager.active_robot
        robot.set_pose(desig.target)


class tiagoMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = RobotManager.active_robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_1_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_2_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = - np.arctan2(pose_in_tilt.position.z,
                                pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_position("head_1_joint")
        current_tilt = robot.get_joint_position("head_2_joint")

        robot.set_joint_position("head_1_joint", new_pan + current_pan)
        robot.set_joint_position("head_2_joint", new_tilt + current_tilt)


class tiagoMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion):
        robot = RobotManager.active_robot
        gripper = desig.gripper
        motion = desig.motion
        for joint, state in RobotDescription.current_robot_description.get_arm_chain(
                desig.gripper).get_static_gripper_state(motion).items():
            robot.set_joint_position(joint, state)


class tiagoDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion):
        robot = RobotManager.active_robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = RobotDescription.current_robot_description.get_camera_frame()
        # should be [0, 0, 1]
        camera_description = RobotDescription.current_robot_description.cameras[
            list(RobotDescription.current_robot_description.cameras.keys())[0]]
        front_facing_axis = camera_description.front_facing_axis

        objects = World.current_world.get_object_by_type(object_type)
        for obj in objects:
            if btr.visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                return obj


class tiagoMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion):
        target = desig.target
        robot = RobotManager.active_robot

        _move_arm_tcp(target, robot, desig.arm)


class tiagoMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion):

        robot = RobotManager.active_robot
        if desig.right_arm_poses:
            robot.set_joint_positions(desig.right_arm_poses)
        if desig.left_arm_poses:
            robot.set_joint_positions(desig.left_arm_poses)


class tiagoMoveJoints(ProcessModule):
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """

    def _execute(self, desig: MoveJointsMotion):
        robot = desig.used_robot

        if robot is None:
            robot = RobotManager.active_robot

        robot.set_joint_positions(dict(zip(desig.names, desig.positions)))


class tiagoWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, World.current_world.objects))[0]


class tiagoOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, RobotManager.active_robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[1] - 0.05)


class tiagoClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion):
        part_of_object = desig.object_part.world_object

        container_joint = part_of_object.find_joint_above_link(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, RobotManager.active_robot, desig.arm)

        desig.object_part.world_object.set_joint_position(container_joint,
                                                          part_of_object.get_joint_limits(
                                                              container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: Arms) -> None:
    gripper = RobotDescription.current_robot_description.get_arm_chain(arm).get_tool_frame()

    joints = RobotDescription.current_robot_description.get_arm_chain(arm).joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv)


###########################################################
########## Process Modules for the Real tiago ###############
###########################################################


class tiagoNavigationReal(ProcessModule):
    """
    Process module for the real tiago that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, RobotDescription.current_robot_description.base_link, "map")


class tiagoMoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion):
        target = desig.target
        robot = RobotManager.active_robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_1_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_2_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_position("head_1_joint")
        current_tilt = robot.get_joint_position("head_2_joint")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({"head_1_joint": new_pan + current_pan,
                                    "head_2_joint": new_tilt + current_tilt})


class tiagoDetectingReal(ProcessModule):
    """
    Process Module for the real tiago that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, designator: DetectingMotion) -> Any:
        query_result = query_object(ObjectDesignatorDescription(types=[designator.object_type]))
        # print(query_result)
        obj_pose = query_result["ClusterPoseBBAnnotator"]

        lt = LocalTransformer()
        obj_pose = lt.transform_pose(obj_pose, RobotManager.active_robot.get_link_tf_frame("torso_lift_link"))
        obj_pose.orientation = [0, 0, 0, 1]
        obj_pose.position.x += 0.05

        bullet_obj = World.current_world.get_object_by_type(designator.object_type)
        if bullet_obj:
            bullet_obj[0].set_pose(obj_pose)
            return bullet_obj[0]
        elif designator.object_type == ObjectType.JEROEN_CUP:
            cup = Object("cup", ObjectType.JEROEN_CUP, "jeroen_cup.stl", pose=obj_pose)
            return cup
        elif designator.object_type == ObjectType.BOWL:
            bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=obj_pose)
            return bowl

        return bullet_obj[0]


class tiagoMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real tiago while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")

        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, RobotDescription.current_robot_description.get_arm_chain(
            designator.arm).get_tool_frame(),
                                       "torso_lift_link")
        # robot_description.base_link)


class tiagoMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real tiago to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        if designator.right_arm_poses:
            joint_goals.update(designator.right_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class tiagoMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class tiagoMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real tiago, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        def activate_callback():
            rospy.loginfo("Started gripper Movement")

        def done_callback(state, result):
            rospy.loginfo(f"Reached goal {designator.motion}: {result.reached_goal}")

        def feedback_callback(msg):
            pass

        goal = tiagoGripperCommandGoal()
        goal.command.position = 0.0 if designator.motion == "close" else 0.1
        goal.command.max_effort = 50.0
        controller_topic = "r_gripper_controller/gripper_action" if designator.gripper == "right" else "l_gripper_controller/gripper_action"
        client = actionlib.SimpleActionClient(controller_topic, tiagoGripperCommandAction)
        rospy.loginfo("Waiting for action server")
        client.wait_for_server()
        client.send_goal(goal, active_cb=activate_callback, done_cb=done_callback, feedback_cb=feedback_callback)
        wait = client.wait_for_result()


class tiagoOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion) -> Any:
        giskard.achieve_open_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class tiagoCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion) -> Any:
        giskard.achieve_close_container_goal(
            RobotDescription.current_robot_description.get_arm_chain(designator.arm).get_tool_frame(),
            designator.object_part.name)


class tiagoManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("tiago_dual")
        self._navigate_lock = Lock()
        self._pick_up_lock = Lock()
        self._place_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return tiagoNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return tiagoNavigationReal(self._navigate_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return tiagoMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return tiagoMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return tiagoDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return tiagoDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return tiagoMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return tiagoMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return tiagoMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return tiagoMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED or ProcessModuleManager.execution_type == ExecutionType.REAL:
            return tiagoWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return tiagoMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return tiagoMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return tiagoMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return tiagoMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return tiagoOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return tiagoOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return tiagoClose(self._close_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return tiagoCloseReal(self._close_lock)
