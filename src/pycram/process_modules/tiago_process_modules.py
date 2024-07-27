from abc import ABC
from threading import Lock
from typing import Any

import actionlib

import pycram.bullet_world_reasoning as btr
import numpy as np
import time
import rospy
import pybullet as p

from ..plan_failures import EnvironmentManipulationImpossible
from ..process_module import ProcessModule, ProcessModuleManager
from ..bullet_world import BulletWorld, Object
from ..helper import transform
from ..external_interfaces.ik import request_ik, IKError
from ..helper import _transform_to_torso, _apply_ik, calculate_wrist_tool_offset, inverseTimes
from ..local_transformer import LocalTransformer
from ..designators.motion_designator import *
from ..enums import JointType, ObjectType
from ..external_interfaces import giskard
from ..external_interfaces.robokudo import query

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

    robot = BulletWorld.robot
    if arm == "right":
        for joint, pose in get_robot_description().get_static_joint_chain("right", "park").items():
            robot.set_joint_state(joint, pose)
    if arm == "left":
        for joint, pose in get_robot_description().get_static_joint_chain("left", "park").items():
            robot.set_joint_state(joint, pose)


class tiagoNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion.Motion):
        robot = BulletWorld.robot
        robot.set_pose(desig.target)


class tiagoPickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """

    def _execute(self, desig: PickUpMotion.Motion):
        object = desig.object_desig.bullet_world_object
        robot = BulletWorld.robot
        grasp = get_robot_description().grasps.get_orientation_for_grasp(desig.grasp)
        target = object.get_pose()
        target.orientation.x = grasp[0]
        target.orientation.y = grasp[1]
        target.orientation.z = grasp[2]
        target.orientation.w = grasp[3]

        arm = desig.arm

        _move_arm_tcp(target, robot, arm)
        tool_frame = get_robot_description().get_tool_frame(arm)
        robot.attach(object, tool_frame)


class tiagoPlace(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """

    def _execute(self, desig: PlaceMotion.Motion):
        """

        :param desig: A PlaceMotion
        :return:
        """
        object = desig.object.bullet_world_object
        robot = BulletWorld.robot
        arm = desig.arm

        # Transformations such that the target position is the position of the object and not the tcp
        object_pose = object.get_pose()
        local_tf = LocalTransformer()
        tcp_to_object = local_tf.transform_pose(object_pose, robot.get_link_tf_frame(get_robot_description().get_tool_frame(arm)))
        target_diff = desig.target.to_transform("target").inverse_times(tcp_to_object.to_transform("object")).to_pose()

        _move_arm_tcp(target_diff, robot, arm)
        robot.detach(object)


class tiagoMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_1_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_2_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = - np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_state("head_1_joint")
        current_tilt = robot.get_joint_state("head_2_joint")

        robot.set_joint_state("head_1_joint", new_pan + current_pan)
        robot.set_joint_state("head_2_joint", new_tilt + current_tilt)


class tiagoMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion.Motion):
        robot = BulletWorld.robot
        gripper = desig.gripper
        motion = desig.motion
        for joint, state in get_robot_description().get_static_gripper_chain(gripper, motion).items():
            robot.set_joint_state(joint, state)


class tiagoDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion.Motion):
        robot = BulletWorld.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = get_robot_description().get_camera_frame()
        # should be [0, 0, 1]
        front_facing_axis = get_robot_description().front_facing_axis

        if desig.technique == 'all':
            objects = BulletWorld.current_bullet_world.get_all_objets_not_robot()
        elif desig.technique == 'human':
            human = []
            human.append(Object("human", ObjectType.HUMAN, "human_male.stl", pose=Pose([0, 0, 0])))
            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
            for i, obj in enumerate(human):
                object_dict[obj.name] = obj
            return object_dict

        else:
            objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)

        object_dict = {}

        perceived_objects = []
        for obj in objects:
            perceived_objects.append(ObjectDesignatorDescription.Object(obj.name, obj.type, obj))

            #todo: commented out since the visualisation is not working good bc of rendering one object
            #if btr.visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):

        # Iterate over the list of objects and store each one in the dictionary
        for i, obj in enumerate(perceived_objects):
            object_dict[obj.name] = obj

        return object_dict


class tiagoMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        _move_arm_tcp(target, robot, desig.arm)


class tiagoMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion.Motion):

        robot = BulletWorld.robot
        if desig.right_arm_poses:
            robot.set_joint_states(desig.right_arm_poses)
        if desig.left_arm_poses:
            robot.set_joint_states(desig.left_arm_poses)


class tiagoMoveJoints(ProcessModule):
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """
    def _execute(self, desig: MoveJointsMotion.Motion):
        robot = BulletWorld.robot
        robot.set_joint_states(dict(zip(desig.names, desig.positions)))


class tiagoWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion.Motion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class tiagoOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[1])


class tiagoClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(
                                                                  container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: str) -> None:
    gripper = get_robot_description().get_tool_frame(arm)

    joints = get_robot_description().chains[arm].joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv, joints)


###########################################################
########## Process Modules for the Real tiago ###############
###########################################################


class tiagoNavigationReal(ProcessModule):
    """
    Process module for the real tiago that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion.Motion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, get_robot_description().base_link, "map")


class tiagoPickUpReal(ProcessModule):

    def _execute(self, designator: PickUpMotion.Motion) -> Any:
        pass


class tiagoPlaceReal(ProcessModule):

    def _execute(self, designator: MotionDesignatorDescription.Motion) -> Any:
        pass


class tiagoMoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_1_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_2_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_state("head_1_joint")
        current_tilt = robot.get_joint_state("head_2_joint")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal({"head_1_joint": new_pan + current_pan,
                                    "head_2_joint": new_tilt + current_tilt})


class tiagoDetectingReal(ProcessModule):
    """
    Process Module for the real tiago that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, designator: DetectingMotion.Motion) -> Any:
        query_result = query(ObjectDesignatorDescription(types=[designator.object_type]))
        # print(query_result)
        obj_pose = query_result["ClusterPoseBBAnnotator"]

        lt = LocalTransformer()
        obj_pose = lt.transform_pose(obj_pose, BulletWorld.robot.get_link_tf_frame("torso_lift_link"))
        obj_pose.orientation = [0, 0, 0, 1]
        obj_pose.position.x += 0.05

        bullet_obj = BulletWorld.current_bullet_world.get_objects_by_type(designator.object_type)
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

    def _execute(self, designator: MoveTCPMotion.Motion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")

        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, get_robot_description().get_tool_frame(designator.arm),
                                       get_robot_description().base_link)


class tiagoMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real tiago to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion.Motion) -> Any:
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

    def _execute(self, designator: MoveJointsMotion.Motion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class tiagoMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real tiago, gripper uses an action server for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion.Motion) -> Any:
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

    def _execute(self, designator: OpeningMotion.Motion) -> Any:
        giskard.achieve_open_container_goal(get_robot_description().get_tool_frame(designator.arm),
                                            designator.object_part.name)


class tiagoCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion.Motion) -> Any:
        giskard.achieve_close_container_goal(get_robot_description().get_tool_frame(designator.arm),
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
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoNavigationReal(self._navigate_lock)

    def pick_up(self):
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoPickUp(self._pick_up_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoPickUpReal(self._pick_up_lock)

    def place(self):
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoPlace(self._place_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoPlaceReal(self._place_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoDetectingReal(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return tiagoWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return tiagoClose(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return tiagoCloseReal(self._close_lock)
