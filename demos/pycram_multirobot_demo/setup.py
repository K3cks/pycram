from enum import Enum, auto

import rospy

from pycram.datastructures.enums import ObjectType, Arms
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import NavigateAction, ParkArmsAction, MoveTorsoAction
from pycram.world_concepts.world_object import Object


class ROBOTS(Enum):
    PANDA = 'panda'
    PR2 = 'pr2'
    HSRB = 'hsrb'
    TIAGO = 'tiago'
    BOXY = 'boxy'
    UR5 = 'ur5'


class ENVIRONMENTS(Enum):
    APARTMENT = auto()
    KITCHEN = auto()


def create_robot(robot: ROBOTS, pose=None):
    if pose is None:
        pose = Pose([0, 1, 0])

    # Not working, flying randomly around
    if robot == ROBOTS.PR2:
        return Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=pose)

    # Not able to launch yet
    elif robot == ROBOTS.HSRB:
        return Object("hsrb", ObjectType.ROBOT, "hsrb.urdf", pose=pose)

    # Somewhat works, arms are not being affected by gravity
    elif robot == ROBOTS.TIAGO:
        return Object("tiago_dual", ObjectType.ROBOT, "tiago_dual.urdf", pose=pose)

    # Torso falling down, arms are slowly moving downwards
    elif robot == ROBOTS.BOXY:
        return Object("boxy", ObjectType.ROBOT, "boxy.urdf", pose=pose)

    # Not fully tested yet, does not explode like pr2
    elif robot == ROBOTS.UR5:
        return Object("ur5", ObjectType.ROBOT, "ur5_robotiq.urdf", pose=pose)

    else:
        raise Exception("No known Robot defined for world")


def set_environment(environment: ENVIRONMENTS):
    if environment == ENVIRONMENTS.APARTMENT:
        return Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")
    elif environment == ENVIRONMENTS.KITCHEN:
        return Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")

    else:
        raise Exception("No known Environment defined for world")


def set_active_robot(robot: ROBOTS):
    robot_topic = 'multirobot_description/' + robot.name

    try:
        active_robot = rospy.get_param(robot_topic)
        rospy.set_param('robot_description', active_robot)
    except:
        rospy.logerr(f'No topic named {robot_topic} found')


def actions(park=False, torso=False, navigate=False):
    if park:
        print("Parking arms")
        rospy.sleep(2)
        ParkArmsAction([Arms.BOTH]).resolve().perform()

    if torso:
        print("Moving Torso")
        rospy.sleep(2)
        MoveTorsoAction([0.33]).resolve().perform()

    if navigate:
        print("Navigating")
        rospy.sleep(2)
        NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    print("done")
