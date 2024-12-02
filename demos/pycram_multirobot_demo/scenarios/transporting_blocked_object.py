import rospy

from demos.pycram_multirobot_demo.setup.actions import actions
from demos.utils.enums import ROBOTS, ENVIRONMENTS
from demos.utils.launcher import launch_robot
from demos.utils.object_spawner import set_environment, create_robot
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, Arms, Grasp, WorldMode
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import NavigateAction, PickUpAction, PlaceAction
from pycram.designators.object_designator import BelieveObject
from pycram.multirobot import RobotManager
from pycram.process_module import simulated_robot
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


def pr2_pickup_object(robot, object):
    with simulated_robot(robot):
        actions(park=True)

        NavigateAction(target_locations=[Pose([1.3, 3, 0], orientation=[0, 0, 1, 0])]).resolve().perform()

        PickUpAction(object_designator_description=object,
                     arms=[Arms.LEFT],
                     grasps=[Grasp.FRONT]).resolve().perform()

        actions(park=True)

        NavigateAction(target_locations=[Pose([1.9, 3, 0], orientation=[0, 0, 0, 1])]).resolve().perform()

        actions(park=True)


def block_object(object_desig, robot):
    RobotManager.block_object(object_desig=object_desig, robot_name=robot.name)


def release_object(object_desig):
    RobotManager.release_object(object_desig=object_desig)


def tiago_pickup_object(robot):
    with simulated_robot(robot):
        actions(park=True)

        # NavigateAction(target_locations=[Pose([3.5, 3, 0], orientation=[0, 0, 1, 0])]).resolve().perform()
        NavigateAction(target_locations=[Pose([3.6, 3, 0], [0, 0, 1, 0])]).resolve().perform()

        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.75, 3, 1.02], orientation=[0, 0, 1, 0]),
                      color=Color(1, 0, 0, 1))

        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([3.3, 3.30, 0.62], orientation=[0, 0, 1, 0]),
                      color=Color(1, 0, 0, 1))

        robot.attach(milk)

        rospy.sleep(2)

        NavigateAction(target_locations=[Pose([4, 4, 0], [0, 0, 0, 1])]).resolve().perform()

        rospy.sleep(2)

        robot.detach(milk)

        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([4.9, 3.8, 0.82], orientation=[0, 0, 1, 0]),
                      color=Color(1, 0, 0, 1))


def transporting_blocked_object(robot_one: ROBOTS, robot_two: ROBOTS, launch_robots=True):
    if launch_robots:
        first_robot_launch = launch_robot(robot_one, use_namespace=True)
        second_robot_launch = launch_robot(robot_two, use_namespace=True)

    pose_pr2 = Pose([1.5, 3, 0])
    pose_tiago = Pose([4, 3, 0])

    # Environment
    current_environment = set_environment(ENVIRONMENTS.APARTMENT_SMALL)
    milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([0.5, 3, 1.02], orientation=[0, 0, 1, 0]),
                  color=Color(1, 0, 0, 1))
    milk_BO = BelieveObject(names=["milk"])
    cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                    pose=Pose([0.5, 3.3, 1.05]), color=Color(0, 1, 0, 1))
    spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([0.4, 3.2, 0.85]),
                   color=Color(0, 0, 1, 1))
    bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([0.5, 3.2, 1.02]),
                  color=Color(1, 1, 0, 1))

    # Spawn Robots
    first_robot = create_robot(robot_one, pose=pose_pr2)
    second_robot = create_robot(robot_two, pose=pose_tiago)
    rospy.sleep(3)

    # Actions
    print(f"{first_robot.name} actions")
    # pr2_pickup_object(robot=first_robot, object=milk_BO)
    block_object(object_desig=milk, robot=robot_one)

    print("pr2 done")
    print("release pr2 object")

    rospy.sleep(2)

    release_object(object_desig=milk)

    rospy.sleep(3)
    print(f"{second_robot.name} actions")
    # tiago_pickup_object(second_robot)
    block_object(object_desig=milk, robot=robot_two)
    print("tiago done")


if __name__ == '__main__':
    r1 = ROBOTS.PR2
    r2 = ROBOTS.TIAGO
    mode = WorldMode.GUI

    world = BulletWorld(mode)
    viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

    transporting_blocked_object(robot_one=r1, robot_two=r2)
