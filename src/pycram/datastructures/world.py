# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from copy import copy

import numpy as np
import rospy
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from typing_extensions import List, Optional, Dict, Tuple, Callable, TYPE_CHECKING, Union

from ..cache_manager import CacheManager
from ..config import world_conf as conf
from ..datastructures.dataclasses import (Color, AxisAlignedBoundingBox, CollisionCallbacks,
                                          MultiBody, VisualShape, BoxVisualShape, CylinderVisualShape,
                                          SphereVisualShape,
                                          CapsuleVisualShape, PlaneVisualShape, MeshVisualShape,
                                          ObjectState, WorldState, ClosestPointsList,
                                          ContactPointsList, VirtualMoveBaseJoints)
from ..datastructures.enums import JointType, ObjectType, WorldMode, Arms
from ..datastructures.pose import Pose, Transform
from ..datastructures.world_entity import StateEntity
from ..exceptions import ProspectionObjectNotFound, WorldObjectNotFound
from ..local_transformer import LocalTransformer
from ..robot_description import RobotDescription
from ..validation.goal_validator import (MultiPoseGoalValidator,
                                         PoseGoalValidator, JointPositionGoalValidator,
                                         MultiJointPositionGoalValidator, GoalValidator,
                                         validate_joint_position, validate_multiple_joint_positions,
                                         validate_object_pose)
from ..world_concepts.constraints import Constraint
from ..world_concepts.event import Event

if TYPE_CHECKING:
    from ..world_concepts.world_object import Object
    from ..description import Link, Joint


class World(StateEntity, ABC):
    """
    The World Class represents the physics Simulation and belief state, it is the main interface for reasoning about
    the World. This is implemented as a singleton, the current World can be accessed via the static variable
    current_world which is managed by the World class itself.
    """

    simulation_frequency: float
    """
    Global reference for the simulation frequency (Hz), used in calculating the equivalent real time in the simulation.
    """

    current_world: Optional[World] = None
    """
        Global reference to the currently used World, usually this is the
        graphical one. However, if you are inside a UseProspectionWorld() environment the current_world points to the
        prospection world. In this way you can comfortably use the current_world, which should point towards the World
        used at the moment.
    """

    robot: Optional[Object] = None
    """
    Global reference to the spawned Object that represents the robot. The robot is identified by checking the name in
     the URDF with the name of the URDF on the parameter server. 
    """

    resources_path = conf.resources_path
    """
    Global reference for the resources path, this is used to search for the description files of the robot and
     the objects.
    """

    data_directory: List[str] = [resources_path]
    """
    Global reference for the data directories, this is used to search for the description files of the robot,
     the objects, and the cached files.
    """

    cache_dir: str = conf.cache_dir
    """
    Global reference for the cache directory, this is used to cache the description files of the robot and the objects.
    """

    cache_manager: CacheManager = CacheManager(cache_dir, data_directory)
    """
    Global reference for the cache manager, this is used to cache the description files of the robot and the objects.
    """

    prospection_world_prefix: str = conf.prospection_world_prefix
    """
    The prefix for the prospection world name.
    """

    let_pycram_move_attached_objects: bool = conf.JobHandling.let_pycram_move_attached_objects
    let_pycram_handle_spawning: bool = conf.JobHandling.let_pycram_handle_spawning
    let_pycram_handle_world_sync: bool = conf.JobHandling.let_pycram_handle_world_sync
    """
    Whether to let PyCRAM handle the movement of attached objects, the spawning of objects,
     and the world synchronization.
    """

    update_poses_from_sim_on_get: bool = conf.update_poses_from_sim_on_get
    """
    Whether to update the poses from the simulator when getting the object poses.
    """

    acceptable_position_error: float = conf.ErrorTolerance.acceptable_position_error
    acceptable_orientation_error: float = conf.ErrorTolerance.acceptable_orientation_error
    acceptable_pose_error: Tuple[float, float] = conf.ErrorTolerance.acceptable_pose_error
    acceptable_revolute_joint_position_error: float = conf.ErrorTolerance.acceptable_revolute_joint_position_error
    acceptable_prismatic_joint_position_error: float = conf.ErrorTolerance.acceptable_prismatic_joint_position_error
    use_percentage_of_goal: bool = conf.ErrorTolerance.use_percentage_of_goal
    acceptable_percentage_of_goal: float = conf.ErrorTolerance.acceptable_percentage_of_goal
    """
    The acceptable error for the position and orientation of an object/link.
    """

    raise_goal_validator_error: Optional[bool] = conf.ErrorTolerance.raise_goal_validator_error
    """
    Whether to raise an error if the goals are not achieved.
    """

    def __init__(self, mode: WorldMode, is_prospection_world: bool, simulation_frequency: float,
                 **config_kwargs):
        """
        Creates a new simulation, the mode decides if the simulation should be a rendered window or just run in the
        background. There can only be one rendered simulation.
        The World object also initializes the Events for attachment, detachment and for manipulating the world.

        :param mode: Can either be "GUI" for rendered window or "DIRECT" for non-rendered. The default parameter is
         "GUI"
        :param is_prospection_world: For internal usage, decides if this World should be used as a prospection world.
        :param simulation_frequency: The frequency of the simulation in Hz.
        :param config_kwargs: Additional configuration parameters.
        """

        StateEntity.__init__(self)

        # Parse config_kwargs
        for key, value in config_kwargs.items():
            setattr(self, key, value)

        GoalValidator.raise_error = World.raise_goal_validator_error
        World.simulation_frequency = simulation_frequency

        if World.current_world is None:
            World.current_world = self

        self.object_lock: threading.Lock = threading.Lock()

        self.id: Optional[int] = -1
        # This is used to connect to the physics server (allows multiple clients)

        self._init_world(mode)

        self.objects: List[Object] = []
        # List of all Objects in the World

        self.is_prospection_world: bool = is_prospection_world
        self._init_and_sync_prospection_world()

        self.local_transformer = LocalTransformer()
        self._update_local_transformer_worlds()

        self.mode: WorldMode = mode
        # The mode of the simulation, can be "GUI" or "DIRECT"

        self.coll_callbacks: Dict[Tuple[Object, Object], CollisionCallbacks] = {}

        self._init_events()

        self._current_state: Optional[WorldState] = None

        self._init_goal_validators()

    @property
    def robot_description(self) -> RobotDescription:
        """
        Returns the current robot description.
        """
        return RobotDescription.current_robot_description

    @property
    def robot_has_actuators(self) -> bool:
        """
        Returns whether the robot has actuators.
        """
        return self.robot_description.has_actuators

    def get_actuator_for_joint(self, joint: Joint) -> str:
        """
        Get the actuator name for a given joint.
        """
        return self.robot_joint_actuators[joint.name]

    def joint_has_actuator(self, joint: Joint) -> bool:
        """
        Returns whether the joint has an actuator.
        """
        return joint.name in self.robot_joint_actuators

    @property
    def robot_joint_actuators(self) -> Dict[str, str]:
        """
        Returns the joint actuators of the robot.
        """
        return self.robot_description.joint_actuators

    def _init_goal_validators(self):
        """
        Initializes the goal validators for the World objects' poses, positions, and orientations.
        """

        # Objects Pose goal validators
        self.pose_goal_validator = PoseGoalValidator(self.get_object_pose, self.acceptable_pose_error,
                                                     self.acceptable_percentage_of_goal)
        self.multi_pose_goal_validator = MultiPoseGoalValidator(
            lambda x: list(self.get_multiple_object_poses(x).values()),
            self.acceptable_pose_error, self.acceptable_percentage_of_goal)

        # Joint Goal validators
        self.joint_position_goal_validator = JointPositionGoalValidator(
            self.get_joint_position,
            acceptable_revolute_joint_position_error=self.acceptable_revolute_joint_position_error,
            acceptable_prismatic_joint_position_error=self.acceptable_prismatic_joint_position_error,
            acceptable_percentage_of_goal_achieved=self.acceptable_percentage_of_goal)
        self.multi_joint_position_goal_validator = MultiJointPositionGoalValidator(
            lambda x: list(self.get_multiple_joint_positions(x).values()),
            acceptable_revolute_joint_position_error=self.acceptable_revolute_joint_position_error,
            acceptable_prismatic_joint_position_error=self.acceptable_prismatic_joint_position_error,
            acceptable_percentage_of_goal_achieved=self.acceptable_percentage_of_goal)

    def check_object_exists(self, obj: Object):
        """
        Checks if the object exists in the simulator and issues a warning if it does not.
        :param obj: The object to check.
        """
        return obj not in self.objects

    @abstractmethod
    def _init_world(self, mode: WorldMode):
        """
        Initializes the physics simulation.
        """
        raise NotImplementedError

    def _init_events(self):
        """
        Initializes dynamic events that can be used to react to changes in the World.
        """
        self.detachment_event: Event = Event()
        self.attachment_event: Event = Event()
        self.manipulation_event: Event = Event()

    def _init_and_sync_prospection_world(self):
        """
        Initializes the prospection world and the synchronization between the main and the prospection world.
        """
        self._init_prospection_world()
        self._sync_prospection_world()

    def _update_local_transformer_worlds(self):
        """
        Updates the local transformer worlds with the current world and prospection world.
        """
        self.local_transformer.world = self
        self.local_transformer.prospection_world = self.prospection_world

    def _init_prospection_world(self):
        """
        Initializes the prospection world, if this is a prospection world itself it will not create another prospection,
        world, but instead set the prospection world to None, else it will create a prospection world.
        """
        if self.is_prospection_world:  # then no need to add another prospection world
            self.prospection_world = None
        else:
            self.prospection_world: World = self.__class__(WorldMode.DIRECT,
                                                           True,
                                                           World.simulation_frequency)

    def _sync_prospection_world(self):
        """
        Synchronizes the prospection world with the main world, this means that every object in the main world will be
        added to the prospection world and vice versa.
        """
        if self.is_prospection_world:  # then no need to add another prospection world
            self.world_sync = None
        else:
            self.world_sync: WorldSync = WorldSync(self, self.prospection_world)
            self.pause_world_sync()
            self.world_sync.start()

    def update_cache_dir_with_object(self, path: str, ignore_cached_files: bool,
                                     obj: Object) -> str:
        """
        Updates the cache directory with the given object.

        :param path: The path to the object.
        :param ignore_cached_files: If the cached files should be ignored.
        :param obj: The object to be added to the cache directory.
        """
        return self.cache_manager.update_cache_dir_with_object(path, ignore_cached_files, obj.description, obj.name)

    @property
    def simulation_time_step(self):
        """
        The time step of the simulation in seconds.
        """
        return 1 / World.simulation_frequency

    @abstractmethod
    def load_object_and_get_id(self, path: Optional[str] = None, pose: Optional[Pose] = None,
                               obj_type: Optional[ObjectType] = None) -> int:
        """
        Loads a description file (e.g. URDF) at the given pose and returns the id of the loaded object.

        :param path: The path to the description file, if None the description file is assumed to be already loaded.
        :param pose: The pose at which the object should be loaded.
        :param obj_type: The type of the object.
        :return: The id of the loaded object.
        """
        pass

    def get_object_names(self) -> List[str]:
        """
        Returns the names of all objects in the World.

        :return: A list of object names.
        """
        return [obj.name for obj in self.objects]

    def get_object_by_name(self, name: str) -> Optional[Object]:
        """
        Return the object with the given name. If there is no object with the given name, None is returned.

        :param name: The name of the returned Objects.
        :return: The object with the given name, if there is one.
        """

        matching_objects = list(filter(lambda obj: obj.name == name, self.objects))
        return matching_objects[0] if len(matching_objects) > 0 else None

    def get_object_by_type(self, obj_type: ObjectType) -> List[Object]:
        """
        Returns a list of all Objects which have the type 'obj_type'.

        :param obj_type: The type of the returned Objects.
        :return: A list of all Objects that have the type 'obj_type'.
        """
        return list(filter(lambda obj: obj.obj_type == obj_type, self.objects))

    def get_object_by_id(self, obj_id: int) -> Object:
        """
        Returns the single Object that has the unique id.

        :param obj_id: The unique id for which the Object should be returned.
        :return: The Object with the id 'id'.
        """
        return list(filter(lambda obj: obj.id == obj_id, self.objects))[0]

    @abstractmethod
    def remove_object_by_id(self, obj_id: int) -> None:
        """
        Removes the object with the given id from the world.

        :param obj_id: The unique id of the object to be removed.
        """
        pass

    @abstractmethod
    def remove_object_from_simulator(self, obj: Object) -> None:
        """
        Removes an object from the physics simulator.

        :param obj: The object to be removed.
        """
        pass

    def remove_object(self, obj: Object) -> None:
        """
        Removes this object from the current world.
        For the object to be removed it has to be detached from all objects it
        is currently attached to. After this is done a call to world remove object is done
        to remove this Object from the simulation/world.

        :param obj: The object to be removed.
        """
        self.object_lock.acquire()

        obj.detach_all()
        if obj.name != "floor":
            self.objects.remove(obj)
            self.remove_object_from_simulator(obj)
        if World.robot == obj:
            World.robot = None

        self.object_lock.release()

    def add_fixed_constraint(self, parent_link: Link, child_link: Link,
                             child_to_parent_transform: Transform) -> int:
        """
        Creates a fixed joint constraint between the given parent and child links,
        the joint frame will be at the origin of the child link frame, and would have the same orientation
        as the child link frame.

        :param parent_link: The constrained link of the parent object.
        :param child_link: The constrained link of the child object.
        :param child_to_parent_transform: The transform from the child link frame to the parent link frame.
        :return: The unique id of the created constraint.
        """

        constraint = Constraint(parent_link=parent_link,
                                child_link=child_link,
                                _type=JointType.FIXED,
                                axis_in_child_frame=Point(0, 0, 0),
                                constraint_to_parent=child_to_parent_transform,
                                child_to_constraint=Transform(frame=child_link.tf_frame)
                                )
        constraint_id = self.add_constraint(constraint)
        return constraint_id

    @abstractmethod
    def add_constraint(self, constraint: Constraint) -> int:
        """
        Add a constraint between two objects links so that they become attached for example.

        :param constraint: The constraint data used to create the constraint.
        """
        pass

    @abstractmethod
    def remove_constraint(self, constraint_id) -> None:
        """
        Remove a constraint by its ID.

        :param constraint_id: The unique id of the constraint to be removed.
        """
        pass

    @abstractmethod
    def get_joint_position(self, joint: Joint) -> float:
        """
        Get the position of a joint of an articulated object

        :param joint: The joint to get the position for.
        :return: The joint position as a float.
        """
        pass

    @abstractmethod
    def get_object_joint_names(self, obj: Object) -> List[str]:
        """
        Returns the names of all joints of this object.

        :param obj: The object.
        :return: A list of joint names.
        """
        pass

    @abstractmethod
    def get_link_pose(self, link: Link) -> Pose:
        """
        Get the pose of a link of an articulated object with respect to the world frame.

        :param link: The link as a AbstractLink object.
        :return: The pose of the link as a Pose object.
        """
        pass

    @abstractmethod
    def get_multiple_link_poses(self, links: List[Link]) -> Dict[str, Pose]:
        """
        Get the poses of multiple links of an articulated object with respect to the world frame.

        :param links: The links as a list of AbstractLink objects.
        :return: A dictionary with link names as keys and Pose objects as values.
        """
        pass

    @abstractmethod
    def get_link_position(self, link: Link) -> List[float]:
        """
        Get the position of a link of an articulated object with respect to the world frame.

        :param link: The link as a AbstractLink object.
        :return: The position of the link as a list of floats.
        """
        pass

    @abstractmethod
    def get_link_orientation(self, link: Link) -> List[float]:
        """
        Get the orientation of a link of an articulated object with respect to the world frame.

        :param link: The link as a AbstractLink object.
        :return: The orientation of the link as a list of floats.
        """
        pass

    @abstractmethod
    def get_multiple_link_positions(self, links: List[Link]) -> Dict[str, List[float]]:
        """
        Get the positions of multiple links of an articulated object with respect to the world frame.

        :param links: The links as a list of AbstractLink objects.
        :return: A dictionary with link names as keys and lists of floats as values.
        """
        pass

    @abstractmethod
    def get_multiple_link_orientations(self, links: List[Link]) -> Dict[str, List[float]]:
        """
        Get the orientations of multiple links of an articulated object with respect to the world frame.

        :param links: The links as a list of AbstractLink objects.
        :return: A dictionary with link names as keys and lists of floats as values.
        """
        pass

    @abstractmethod
    def get_object_link_names(self, obj: Object) -> List[str]:
        """
        Returns the names of all links of this object.

        :param obj: The object.
        :return: A list of link names.
        """
        pass

    def simulate(self, seconds: float, real_time: Optional[bool] = False) -> None:
        """
        Simulates Physics in the World for a given amount of seconds. Usually this simulation is faster than real
        time. By setting the 'real_time' parameter this simulation is slowed down such that the simulated time is equal
        to real time.

        :param seconds: The amount of seconds that should be simulated.
        :param real_time: If the simulation should happen in real time or faster.
        """
        self.set_realtime(real_time)
        for i in range(0, int(seconds * self.simulation_frequency)):
            curr_time = rospy.Time.now()
            self.step()
            for objects, callbacks in self.coll_callbacks.items():
                contact_points = self.get_contact_points_between_two_objects(objects[0], objects[1])
                if len(contact_points) > 0:
                    callbacks.on_collision_cb()
                elif callbacks.no_collision_cb is not None:
                    callbacks.no_collision_cb()
            if real_time:
                loop_time = rospy.Time.now() - curr_time
                time_diff = self.simulation_time_step - loop_time.to_sec()
                time.sleep(max(0, time_diff))
        self.update_all_objects_poses()

    def update_all_objects_poses(self) -> None:
        """
        Updates the positions of all objects in the world.
        """
        for obj in self.objects:
            obj.update_pose()

    @abstractmethod
    def get_object_pose(self, obj: Object) -> Pose:
        """
        Get the pose of an object in the world frame from the current object pose in the simulator.
        """
        pass

    @abstractmethod
    def get_multiple_object_poses(self, objects: List[Object]) -> Dict[str, Pose]:
        """
        Get the poses of multiple objects in the world frame from the current object poses in the simulator.
        """
        pass

    @abstractmethod
    def get_multiple_object_positions(self, objects: List[Object]) -> Dict[str, List[float]]:
        """
        Get the positions of multiple objects in the world frame from the current object poses in the simulator.
        """
        pass

    @abstractmethod
    def get_object_position(self, obj: Object) -> List[float]:
        """
        Get the position of an object in the world frame from the current object pose in the simulator.
        """
        pass

    @abstractmethod
    def get_multiple_object_orientations(self, objects: List[Object]) -> Dict[str, List[float]]:
        """
        Get the orientations of multiple objects in the world frame from the current object poses in the simulator.
        """
        pass

    @abstractmethod
    def get_object_orientation(self, obj: Object) -> List[float]:
        """
        Get the orientation of an object in the world frame from the current object pose in the simulator.
        """
        pass

    @property
    def robot_virtual_joints(self) -> List[Joint]:
        """
        Returns the virtual joints of the robot.
        """
        return [self.robot.joints[name] for name in self.robot_virtual_joints_names]

    @property
    def robot_virtual_joints_names(self) -> List[str]:
        """
        Returns the virtual joints of the robot.
        """
        return self.robot_description.virtual_move_base_joints.names

    def set_mobile_robot_pose(self, robot_obj: Object, pose: Pose) -> None:
        """
        Set the goal for the move base joints of a mobile robot to reach a target pose. This is used for example when
        the simulator does not support setting the pose of the robot directly (e.g. MuJoCo).
        :param robot_obj: The robot object.
        :param pose: The target pose.
        """
        goal = self.get_move_base_joint_goal(pose)
        robot_obj.set_multiple_joint_positions(goal)

    def get_move_base_joint_goal(self, pose: Pose) -> Dict[str, float]:
        """
        Get the goal for the move base joints of a mobile robot to reach a target pose.
        param pose: The target pose.
        return: The goal for the move base joints.
        """
        position_diff = self.get_position_diff(self.robot.get_position_as_list(), pose.position_as_list())[:2]
        target_angle = self.get_z_angle(pose.orientation_as_list())
        # Get the joints of the base link
        move_base_joints = self.get_move_base_joints()
        return {move_base_joints.translation_x: position_diff[0],
                move_base_joints.translation_y: position_diff[1],
                move_base_joints.angular_z: target_angle}

    def get_move_base_joints(self) -> VirtualMoveBaseJoints:
        """
        Get the move base joints of the robot.

        :return: The move base joints.
        """
        return self.robot_description.virtual_move_base_joints

    @staticmethod
    def get_position_diff(current_position: List[float], target_position: List[float]) -> List[float]:
        """
        Get the difference between two positions.
        param current_position: The current position.
        param target_position: The target position.
        return: The difference between the two positions.
        """
        return [target_position[i] - current_position[i] for i in range(3)]

    @staticmethod
    def get_z_angle(target_quaternion: List[float]) -> float:
        """
        Get the z angle from a quaternion by converting it to euler angles.
        param target_quaternion: The target quaternion.
        return: The z angle.
        """
        return euler_from_quaternion(target_quaternion)[2]

    @abstractmethod
    def perform_collision_detection(self) -> None:
        """
        Checks for collisions between all objects in the World and updates the contact points.
        """
        pass

    @abstractmethod
    def get_object_contact_points(self, obj: Object) -> ContactPointsList:
        """
        Returns a list of contact points of this Object with all other Objects.

        :param obj: The object.
        :return: A list of all contact points with other objects
        """
        pass

    @abstractmethod
    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> ContactPointsList:
        """
        Returns a list of contact points between obj_a and obj_b.

        :param obj1: The first object.
        :param obj2: The second object.
        :return: A list of all contact points between the two objects.
        """
        pass

    def get_object_closest_points(self, obj: Object, max_distance: float) -> ClosestPointsList:
        """
        Returns the closest points of this object with all other objects in the world.

        :param obj: The object.
        :param max_distance: The maximum distance between the points.
        :return: A list of the closest points.
        """
        all_obj_closest_points = [self.get_closest_points_between_objects(obj, other_obj, max_distance) for other_obj in
                                  self.objects
                                  if other_obj != obj]
        return ClosestPointsList([point for closest_points in all_obj_closest_points for point in closest_points])

    def get_closest_points_between_objects(self, object_a: Object, object_b: Object, max_distance: float) \
            -> ClosestPointsList:
        """
        Returns the closest points between two objects.

        :param object_a: The first object.
        :param object_b: The second object.
        :param max_distance: The maximum distance between the points.
        :return: A list of the closest points.
        """
        raise NotImplementedError

    @validate_joint_position
    @abstractmethod
    def reset_joint_position(self, joint: Joint, joint_position: float) -> bool:
        """
        Reset the joint position instantly without physics simulation
        NOTE: It is recommended to use the validate_joint_position decorator to validate the joint position for
        the implementation of this method.
        :param joint: The joint to reset the position for.
        :param joint_position: The new joint pose.
        :return: True if the reset was successful, False otherwise
        """
        pass

    @validate_multiple_joint_positions
    @abstractmethod
    def set_multiple_joint_positions(self, joint_positions: Dict[Joint, float]) -> bool:
        """
        Set the positions of multiple joints of an articulated object.
        NOTE: It is recommended to use the validate_multiple_joint_positions decorator to validate the
         joint positions for the implementation of this method.
        :param joint_positions: A dictionary with joint objects as keys and joint positions as values.
        :return: True if the set was successful, False otherwise.
        """
        pass

    @abstractmethod
    def get_multiple_joint_positions(self, joints: List[Joint]) -> Dict[str, float]:
        """
        Get the positions of multiple joints of an articulated object.
        """
        pass

    @validate_object_pose
    @abstractmethod
    def reset_object_base_pose(self, obj: Object, pose: Pose) -> bool:
        """
        Reset the world position and orientation of the base of the object instantaneously,
        not through physics simulation. (x,y,z) position vector and (x,y,z,w) quaternion orientation.
        NOTE: It is recommended to use the validate_object_pose decorator to validate the object pose for the
        implementation of this method.
        :param obj: The object.
        :param pose: The new pose as a Pose object.
        :return: True if the reset was successful, False otherwise.
        """
        pass

    @abstractmethod
    def reset_multiple_objects_base_poses(self, objects: Dict[Object, Pose]) -> bool:
        """
        Reset the world position and orientation of the base of multiple objects instantaneously,
        not through physics simulation. (x,y,z) position vector and (x,y,z,w) quaternion orientation.

        :param objects: A dictionary with objects as keys and poses as values.
        :return: True if the reset was successful, False otherwise.
        """
        pass

    @abstractmethod
    def step(self):
        """
        Step the world simulation using forward dynamics
        """
        pass

    def get_arm_tool_frame_link(self, arm: Arms) -> Link:
        """
        Get the tool frame link of the arm of the robot.

        :param arm: The arm for which the tool frame link should be returned.
        :return: The tool frame link of the arm.
        """
        ee_link_name = self.robot_description.get_arm_tool_frame(arm)
        return self.robot.get_link(ee_link_name)

    @abstractmethod
    def set_link_color(self, link: Link, rgba_color: Color):
        """
        Changes the rgba_color of a link of this object, the rgba_color has to be given as Color object.

        :param link: The link which should be colored.
        :param rgba_color: The rgba_color as Color object with RGBA values between 0 and 1.
        """
        pass

    @abstractmethod
    def get_link_color(self, link: Link) -> Color:
        """
        This method returns the rgba_color of this link.

        :param link: The link for which the rgba_color should be returned.
        :return: The rgba_color as Color object with RGBA values between 0 and 1.
        """
        pass

    @abstractmethod
    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        """
        Get the RGBA colors of each link in the object as a dictionary from link name to rgba_color.

        :param obj: The object
        :return: A dictionary with link names as keys and a Color object for each link as value.
        """
        pass

    @abstractmethod
    def get_object_axis_aligned_bounding_box(self, obj: Object) -> AxisAlignedBoundingBox:
        """
        Returns the axis aligned bounding box of this object. The return of this method are two points in
        world coordinate frame which define a bounding box.

        :param obj: The object for which the bounding box should be returned.
        :return: AxisAlignedBoundingBox object containing the min and max points of the bounding box.
        """
        pass

    @abstractmethod
    def get_link_axis_aligned_bounding_box(self, link: Link) -> AxisAlignedBoundingBox:
        """
        Returns the axis aligned bounding box of the link. The return of this method are two points in
        world coordinate frame which define a bounding box.
        """
        pass

    @abstractmethod
    def set_realtime(self, real_time: bool) -> None:
        """
        Enables the real time simulation of Physics in the World. By default, this is disabled and Physics is only
        simulated to reason about it.

        :param real_time: Whether the World should simulate Physics in real time.
        """
        pass

    @abstractmethod
    def set_gravity(self, gravity_vector: List[float]) -> None:
        """
        Sets the gravity that is used in the World. By default, it is set to the gravity on earth ([0, 0, -9.8]).
         Gravity is given as a vector in x,y,z. Gravity is only applied while simulating Physic.

        :param gravity_vector: The gravity vector that should be used in the World.
        """
        pass

    def set_robot_if_not_set(self, robot: Object) -> None:
        """
        Sets the robot if it is not set yet.

        :param robot: The Object reference to the Object representing the robot.
        """
        if not self.robot_is_set():
            self.set_robot(robot)

    @staticmethod
    def set_robot(robot: Union[Object, None]) -> None:
        """
        Sets the global variable for the robot Object This should be set on spawning the robot.

        :param robot: The Object reference to the Object representing the robot.
        """
        World.robot = robot

    @staticmethod
    def robot_is_set() -> bool:
        """
        Returns whether the robot has been set or not.

        :return: True if the robot has been set, False otherwise.
        """
        return World.robot is not None

    def exit(self) -> None:
        """
        Closes the World as well as the prospection world, also collects any other thread that is running.
        """
        self.exit_prospection_world_if_exists()
        self.disconnect_from_physics_server()
        self.reset_robot()
        self.join_threads()
        if World.current_world == self:
            World.current_world = None

    def exit_prospection_world_if_exists(self) -> None:
        """
        Exits the prospection world if it exists.
        """
        if self.prospection_world:
            self.terminate_world_sync()
            self.prospection_world.exit()

    @abstractmethod
    def disconnect_from_physics_server(self) -> None:
        """
        Disconnects the world from the physics server.
        """
        pass

    def reset_current_world(self) -> None:
        """
        Resets the pose of every object in the World to the pose it was spawned in and sets every joint to 0.
        """
        for obj in self.objects:
            obj.set_pose(obj.original_pose)
            obj.set_multiple_joint_positions(dict(zip(list(obj.joint_names), [0] * len(obj.joint_names))))

    def reset_robot(self) -> None:
        """
        Sets the robot class variable to None.
        """
        self.set_robot(None)

    @abstractmethod
    def join_threads(self) -> None:
        """
        Join any running threads. Useful for example when exiting the world.
        """
        pass

    def terminate_world_sync(self) -> None:
        """
        Terminates the world sync thread.
        """
        self.world_sync.terminate = True
        self.resume_world_sync()
        self.world_sync.join()

    def save_state(self, state_id: Optional[int] = None) -> int:
        """
        Returns the id of the saved state of the World. The saved state contains the states of all the objects and
        the state of the physics simulator.

        :return: A unique id of the state
        """
        state_id = self.save_physics_simulator_state()
        self.save_objects_state(state_id)
        self._current_state = WorldState(state_id, self.object_states)
        return super().save_state(state_id)

    @property
    def current_state(self) -> WorldState:
        if self._current_state is None:
            self._current_state = WorldState(self.save_physics_simulator_state(), self.object_states)
        return WorldState(self._current_state.simulator_state_id, self.object_states)

    @current_state.setter
    def current_state(self, state: WorldState) -> None:
        if self.current_state != state:
            self.restore_physics_simulator_state(state.simulator_state_id)
            for obj in self.objects:
                self.get_object_by_name(obj.name).current_state = state.object_states[obj.name]

    @property
    def object_states(self) -> Dict[str, ObjectState]:
        """
        Returns the states of all objects in the World.

        :return: A dictionary with the object id as key and the object state as value.
        """
        return {obj.name: obj.current_state for obj in self.objects}

    @object_states.setter
    def object_states(self, states: Dict[str, ObjectState]) -> None:
        """
        Sets the states of all objects in the World.
        """
        for obj_name, obj_state in states.items():
            self.get_object_by_name(obj_name).current_state = obj_state

    def save_objects_state(self, state_id: int) -> None:
        """
        Saves the state of all objects in the World according to the given state using the unique state id.

        :param state_id: The unique id representing the state.
        """
        for obj in self.objects:
            obj.save_state(state_id)

    @abstractmethod
    def save_physics_simulator_state(self) -> int:
        """
        Saves the state of the physics simulator and returns the unique id of the state.

        :return: The unique id representing the state.
        """
        pass

    @abstractmethod
    def remove_physics_simulator_state(self, state_id: int) -> None:
        """
        Removes the state of the physics simulator with the given id.

        :param state_id: The unique id representing the state.
        """
        pass

    @abstractmethod
    def restore_physics_simulator_state(self, state_id: int) -> None:
        """
        Restores the objects and environment state in the physics simulator according to
         the given state using the unique state id.

        :param state_id: The unique id representing the state.
        """
        pass

    def get_images_for_target(self,
                              target_pose: Pose,
                              cam_pose: Pose,
                              size: Optional[int] = 256) -> List[np.ndarray]:
        """
        Calculates the view and projection Matrix and returns 3 images:

        1. An RGB image
        2. A depth image
        3. A segmentation Mask, the segmentation mask indicates for every pixel the visible Object

        :param target_pose: The pose to which the camera should point.
        :param cam_pose: The pose of the camera.
        :param size: The height and width of the images in pixels.
        :return: A list containing an RGB and depth image as well as a segmentation mask, in this order.
        """
        pass

    def register_two_objects_collision_callbacks(self,
                                                 object_a: Object,
                                                 object_b: Object,
                                                 on_collision_callback: Callable,
                                                 on_collision_removal_callback: Optional[Callable] = None) -> None:
        """
        Registers callback methods for contact between two Objects. There can be a callback for when the two Objects
        get in contact and, optionally, for when they are not in contact anymore.

        :param object_a: An object in the World
        :param object_b: Another object in the World
        :param on_collision_callback: A function that should be called if the objects are in contact
        :param on_collision_removal_callback: A function that should be called if the objects are not in contact
        """
        self.coll_callbacks[(object_a, object_b)] = CollisionCallbacks(on_collision_callback,
                                                                       on_collision_removal_callback)

    @classmethod
    def add_resource_path(cls, path: str) -> None:
        """
        Adds a resource path in which the World will search for files. This resource directory is searched if an
        Object is spawned only with a filename.

        :param path: A path in the filesystem in which to search for files.
        """
        cls.data_directory.append(path)

    def get_prospection_object_for_object(self, obj: Object) -> Object:
        """
        Returns the corresponding object from the prospection world for a given object in the main world.
         If the given Object is already in the prospection world, it is returned.

        :param obj: The object for which the corresponding object in the prospection World should be found.
        :return: The corresponding object in the prospection world.
        """
        with UseProspectionWorld():
            return self.world_sync.get_prospection_object(obj)

    def get_object_for_prospection_object(self, prospection_object: Object) -> Object:
        """
        Returns the corresponding object from the main World for a given
        object in the prospection world. If the  given object is not in the prospection
        world an error will be raised.

        :param prospection_object: The object for which the corresponding object in the main World should be found.
        :return: The object in the main World.
        """
        return self.world_sync.get_world_object(prospection_object)

    def reset_world_and_remove_objects(self, exclude_objects: Optional[List[Object]] = None) -> None:
        """
        Resets the World to the state it was first spawned in and removes all objects from the World.
        :param exclude_objects: A list of objects that should not be removed.
        """
        self.reset_world()
        objs_copy = [obj for obj in self.objects]
        exclude_objects = [] if exclude_objects is None else exclude_objects
        [self.remove_object(obj) for obj in objs_copy if obj not in exclude_objects]

    def reset_world(self, remove_saved_states=True) -> None:
        """
        Resets the World to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and all objects will be set to the position and
        orientation in which they were spawned.

        :param remove_saved_states: If the saved states should be removed.
        """

        if remove_saved_states:
            self.remove_saved_states()

        for obj in self.objects:
            obj.reset(remove_saved_states)

    def remove_saved_states(self) -> None:
        """
        Removes all saved states of the World.
        """
        for state_id in self.saved_states:
            self.remove_physics_simulator_state(state_id)
        super().remove_saved_states()

    def update_transforms_for_objects_in_current_world(self) -> None:
        """
        Updates transformations for all objects that are currently in :py:attr:`~pycram.world.World.current_world`.
        """
        curr_time = rospy.Time.now()
        for obj in list(self.current_world.objects):
            obj.update_link_transforms(curr_time)

    @abstractmethod
    def ray_test(self, from_position: List[float], to_position: List[float]) -> int:
        """ Cast a ray and return the first object hit, if any.

        :param from_position: The starting position of the ray in Cartesian world coordinates.
        :param to_position: The ending position of the ray in Cartesian world coordinates.
        :return: The object id of the first object hit, or -1 if no object was hit.
        """
        pass

    @abstractmethod
    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1) -> List[int]:
        """ Cast a batch of rays and return the result for each of the rays (first object hit, if any. or -1)
         Takes optional argument num_threads to specify the number of threads to use
           to compute the ray intersections for the batch. Specify 0 to let simulator decide, 1 (default) for single
            core execution, 2 or more to select the number of threads to use.

        :param from_positions: The starting positions of the rays in Cartesian world coordinates.
        :param to_positions: The ending positions of the rays in Cartesian world coordinates.
        :param num_threads: The number of threads to use to compute the ray intersections for the batch.
        """
        pass

    def create_visual_shape(self, visual_shape: VisualShape) -> int:
        """
        Creates a visual shape in the physics simulator and returns the unique id of the created shape.

        :param visual_shape: The visual shape to be created, uses the VisualShape dataclass defined in world_dataclasses
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_multi_body_from_visual_shapes(self, visual_shape_ids: List[int], pose: Pose) -> int:
        """
        Creates a multi body from visual shapes in the physics simulator and returns the unique id of the created
        multi body.

        :param visual_shape_ids: The ids of the visual shapes that should be used to create the multi body.
        :param pose: The pose of the origin of the multi body relative to the world frame.
        :return: The unique id of the created multi body.
        """
        # Dummy parameter since these are needed to spawn visual shapes as a multibody.
        num_of_shapes = len(visual_shape_ids)
        link_poses = [Pose() for _ in range(num_of_shapes)]
        link_masses = [1.0 for _ in range(num_of_shapes)]
        link_parent = [0 for _ in range(num_of_shapes)]
        link_joints = [JointType.FIXED.value for _ in range(num_of_shapes)]
        link_collision = [-1 for _ in range(num_of_shapes)]
        link_joint_axis = [Point(1, 0, 0) for _ in range(num_of_shapes)]

        multi_body = MultiBody(base_visual_shape_index=-1, base_pose=pose,
                               link_visual_shape_indices=visual_shape_ids, link_poses=link_poses,
                               link_masses=link_masses,
                               link_inertial_frame_poses=link_poses,
                               link_parent_indices=link_parent, link_joint_types=link_joints,
                               link_joint_axis=link_joint_axis,
                               link_collision_shape_indices=link_collision)
        return self.create_multi_body(multi_body)

    def create_multi_body(self, multi_body: MultiBody) -> int:
        """
        Creates a multi body in the physics simulator and returns the unique id of the created multi body. The multibody
        is created by joining multiple links/shapes together with joints.

        :param multi_body: The multi body to be created, uses the MultiBody dataclass defined in world_dataclasses.
        :return: The unique id of the created multi body.
        """
        raise NotImplementedError

    def create_box_visual_shape(self, shape_data: BoxVisualShape) -> int:
        """
        Creates a box visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the box visual shape to be created, uses the BoxVisualShape
         dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_cylinder_visual_shape(self, shape_data: CylinderVisualShape) -> int:
        """
        Creates a cylinder visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the cylinder visual shape to be created, uses the
         CylinderVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_sphere_visual_shape(self, shape_data: SphereVisualShape) -> int:
        """
        Creates a sphere visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the sphere visual shape to be created, uses the SphereVisualShape
         dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_capsule_visual_shape(self, shape_data: CapsuleVisualShape) -> int:
        """
        Creates a capsule visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the capsule visual shape to be created, uses the
         CapsuleVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_plane_visual_shape(self, shape_data: PlaneVisualShape) -> int:
        """
        Creates a plane visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the plane visual shape to be created, uses the PlaneVisualShape
         dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_mesh_visual_shape(self, shape_data: MeshVisualShape) -> int:
        """
        Creates a mesh visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the mesh visual shape to be created,
        uses the MeshVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def add_text(self, text: str, position: List[float], orientation: Optional[List[float]] = None, size: float = 0.1,
                 color: Optional[Color] = Color(), life_time: Optional[float] = 0,
                 parent_object_id: Optional[int] = None, parent_link_id: Optional[int] = None) -> int:
        """
        Adds text to the world.

        :param text: The text to be added.
        :param position: The position of the text in the world.
        :param orientation: By default, debug text will always face the camera, automatically rotation. By specifying a
         text orientation (quaternion), the orientation will be fixed in world space or local space
          (when parent is specified).
        :param size: The size of the text.
        :param color: The color of the text.
        :param life_time: The lifetime in seconds of the text to remain in the world, if 0 the text will remain in the
         world until it is removed manually.
        :param parent_object_id: The id of the object to which the text should be attached.
        :param parent_link_id: The id of the link to which the text should be attached.
        :return: The id of the added text.
        """
        raise NotImplementedError

    def remove_text(self, text_id: Optional[int] = None) -> None:
        """
        Removes text from the world using the given id. if no id is given all text will be removed.

        :param text_id: The id of the text to be removed.
        """
        raise NotImplementedError

    def enable_joint_force_torque_sensor(self, obj: Object, fts_joint_idx: int) -> None:
        """
        You can enable a joint force/torque sensor in each joint. Once enabled, if you perform
        a simulation step, the get_joint_reaction_force_torque will report the joint reaction forces in
        the fixed degrees of freedom: a fixed joint will measure all 6DOF joint forces/torques.
        A revolute/hinge joint force/torque sensor will measure 5DOF reaction forces along all axis except
        the hinge axis. The applied force by a joint motor is available through get_applied_joint_motor_torque.

        :param obj: The object in which the joint is located.
        :param fts_joint_idx: The index of the joint for which the force torque sensor should be enabled.
        """
        raise NotImplementedError

    def disable_joint_force_torque_sensor(self, obj: Object, joint_id: int) -> None:
        """
        Disables the force torque sensor of a joint.

        :param obj: The object in which the joint is located.
        :param joint_id: The id of the joint for which the force torque sensor should be disabled.
        """
        raise NotImplementedError

    def get_joint_reaction_force_torque(self, obj: Object, joint_id: int) -> List[float]:
        """
        Returns the joint reaction forces and torques of the specified joint.

        :param obj: The object in which the joint is located.
        :param joint_id: The id of the joint for which the force torque should be returned.
        :return: The joint reaction forces and torques of the specified joint.
        """
        raise NotImplementedError

    def get_applied_joint_motor_torque(self, obj: Object, joint_id: int) -> float:
        """
        Returns the applied torque by a joint motor.

        :param obj: The object in which the joint is located.
        :param joint_id: The id of the joint for which the applied motor torque should be returned.
        :return: The applied torque by a joint motor.
        """
        raise NotImplementedError

    def pause_world_sync(self) -> None:
        """
        Pauses the world synchronization.
        """
        self.world_sync.sync_lock.acquire()

    def resume_world_sync(self) -> None:
        """
        Resumes the world synchronization.
        """
        self.world_sync.sync_lock.release()

    def __del__(self):
        self.exit()


class UseProspectionWorld:
    """
    An environment for using the prospection world, while in this environment the :py:attr:`~World.current_world`
    variable will point to the prospection world.

    Example:
        with UseProspectionWorld():
            NavigateAction.Action([[1, 0, 0], [0, 0, 0, 1]]).perform()
    """
    WAIT_TIME_AS_N_SIMULATION_STEPS: int = 20
    """
    The time in simulation steps to wait before switching to the prospection world
    """

    def __init__(self):
        self.prev_world: Optional[World] = None
        # The previous world is saved to restore it after the with block is exited.

    @staticmethod
    def sync_worlds():
        """
        Synchronizes the state of the prospection world with the main world.
        """
        for world_obj, prospection_obj in World.current_world.world_sync.object_mapping.items():
            prospection_obj.current_state = world_obj.current_state

    def __enter__(self):
        """
        This method is called when entering the with block, it will set the current world to the prospection world
        """
        if not World.current_world.is_prospection_world:
            self.prev_world = World.current_world
            World.current_world = World.current_world.prospection_world
            World.current_world.resume_world_sync()
            time.sleep(self.WAIT_TIME_AS_N_SIMULATION_STEPS * World.current_world.simulation_time_step)
            World.current_world.pause_world_sync()

    def __exit__(self, *args):
        """
        This method is called when exiting the with block, it will restore the previous world to be the current world.
        """
        if self.prev_world is not None:
            World.current_world = self.prev_world


class WorldSync(threading.Thread):
    """
    Synchronizes the state between the World and its prospection world.
    Meaning the cartesian and joint position of everything in the prospection world will be
    synchronized with the main World.
    The class provides the possibility to pause the synchronization, this can be used
    if reasoning should be done in the prospection world.
    """

    WAIT_TIME_AS_N_SIMULATION_STEPS = 20
    """
    The time in simulation steps to wait between each iteration of the syncing loop.
    """

    def __init__(self, world: World, prospection_world: World):
        threading.Thread.__init__(self)
        self.world: World = world
        self.prospection_world: World = prospection_world
        self.prospection_world.world_sync = self

        self.terminate: bool = False
        self.pause_sync: bool = False
        # Maps world to prospection world objects
        self.object_to_prospection_object_map: Dict[Object, Object] = {}
        self.prospection_object_to_object_map: Dict[Object, Object] = {}
        self.equal_states = False
        self.sync_lock: threading.Lock = threading.Lock()

    def run(self):
        """
        Main method of the synchronization, this thread runs in a loop until the
        terminate flag is set.
        While this loop runs it continuously checks the cartesian and joint position of
        every object in the World and updates the corresponding object in the
        prospection world.
        """
        while not self.terminate:
            self.sync_lock.acquire()
            self.sync_worlds()
            self.sync_lock.release()
            time.sleep(WorldSync.WAIT_TIME_AS_N_SIMULATION_STEPS * self.world.simulation_time_step)

    def get_world_object(self, prospection_object: Object) -> Object:
        """
        Returns the corresponding object from the main World for a given object in the prospection world.

        :param prospection_object: The object for which the corresponding object in the main World should be found.
        :return: The object in the main World.
        """
        try:
            return self.prospection_object_to_object_map[prospection_object]
        except KeyError:
            if prospection_object in self.world.objects:
                return prospection_object
            raise WorldObjectNotFound(prospection_object)

    def get_prospection_object(self, obj: Object) -> Object:
        """
        Returns the corresponding object from the prospection world for a given object in the main world.

        :param obj: The object for which the corresponding object in the prospection World should be found.
        :return: The corresponding object in the prospection world.
        """
        try:
            return self.object_to_prospection_object_map[obj]
        except KeyError:
            if obj in self.prospection_world.objects:
                return obj
            raise ProspectionObjectNotFound(obj)

    def sync_worlds(self):
        """
        Syncs the prospection world with the main world by adding and removing objects and synchronizing their states.
        """
        self.remove_objects_not_in_world()
        self.add_objects_not_in_prospection_world()
        self.prospection_object_to_object_map = {prospection_obj: obj for obj, prospection_obj in
                                                 self.object_to_prospection_object_map.items()}
        self.sync_objects_states()

    def remove_objects_not_in_world(self):
        """
        Removes all objects that are not in the main world from the prospection world.
        """
        obj_map_copy = copy(self.object_to_prospection_object_map)
        [self.remove_object(obj) for obj in obj_map_copy.keys() if obj not in self.world.objects]

    def add_objects_not_in_prospection_world(self):
        """
        Adds all objects that are in the main world but not in the prospection world to the prospection world.
        """
        [self.add_object(obj) for obj in self.world.objects if obj not in self.object_to_prospection_object_map]

    def add_object(self, obj: Object) -> None:
        """
        Adds an object to the prospection world.

        :param obj: The object to be added.
        """
        self.object_to_prospection_object_map[obj] = obj.copy_to_prospection()

    def remove_object(self, obj: Object) -> None:
        """
        Removes an object from the prospection world.

        :param obj: The object to be removed.
        """
        prospection_obj = self.object_to_prospection_object_map[obj]
        prospection_obj.remove()
        del self.object_to_prospection_object_map[obj]

    def sync_objects_states(self) -> None:
        """
        Synchronizes the state of all objects in the World with the prospection world.
        """
        for world_obj, prospection_obj in self.object_to_prospection_object_map.items():
            prospection_obj.current_state = world_obj.current_state

    def check_for_equal(self) -> bool:
        """
        Checks if both Worlds have the same state, meaning all objects are in the same position.
        This is currently not used, but might be used in the future if synchronization issues worsen.

        :return: True if both Worlds have the same state, False otherwise.
        """
        eql = True
        prospection_names = self.prospection_world.get_object_names()
        eql = eql and [name in prospection_names for name in self.world.get_object_names()]
        eql = eql and len(prospection_names) == len(self.world.get_object_names())
        if not eql:
            return False
        for obj, prospection_obj in self.object_to_prospection_object_map.items():
            eql = eql and obj.get_pose().dist(prospection_obj.get_pose()) < 0.001
        self.equal_states = eql
        return eql
