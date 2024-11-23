from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
from pycram.world_concepts.world_object import Object
from .enums import ROBOTS, ENVIRONMENTS


def create_robot(robot: ROBOTS, pose=None):
    if pose is None:
        pose = Pose([0, 1, 0])

    # Not working, flying randomly around
    if robot == ROBOTS.PR2:
        return Object("pr2", ObjectType.ROBOT, "pr2.urdf", pose=pose)

    # Not able to launch yet
    elif robot == ROBOTS.HSRB:
        if pose is None:
            return Object("hsrb", ObjectType.ROBOT, "hsrb.urdf")

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

    elif robot == ROBOTS.TURTLE:
        return Object("turtlebot", ObjectType.ROBOT, "turtlebot.urdf", pose=pose)

    elif robot == ROBOTS.DONBOT:
        return Object("iai_donbot", ObjectType.ROBOT, "iai_donbot.urdf", pose=pose)

    elif robot == ROBOTS.ARMAR6:
        return Object("armar6", ObjectType.ROBOT, "armar6.urdf", pose=pose)

    elif robot == ROBOTS.STRETCH:
        return Object("stretch", ObjectType.ROBOT, "stretch.urdf", pose=pose)

    elif robot == ROBOTS.ICUB:
        return Object("iCub", ObjectType.ROBOT, "iCub.urdf", pose=pose)

    elif robot == ROBOTS.JUSTIN:
        return Object("rollin_justin", ObjectType.ROBOT, "rollin_justin.urdf", pose=pose)

    else:
        raise Exception("No known Robot defined for world")


def set_environment(environment: ENVIRONMENTS):
    if environment == ENVIRONMENTS.APARTMENT:
        return Object("apartment", ObjectType.ENVIRONMENT, "apartment.urdf")
    elif environment == ENVIRONMENTS.KITCHEN:
        return Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
    elif environment == ENVIRONMENTS.APARTMENT_SMALL:
        return Object("apartment", ObjectType.ENVIRONMENT, "apartment-small.urdf")
    elif environment == ENVIRONMENTS.SUTURO:
        return Object("suturo_environment", ObjectType.ENVIRONMENT, "suturo_environment.urdf")

    else:
        raise Exception("No known Environment defined for world")
