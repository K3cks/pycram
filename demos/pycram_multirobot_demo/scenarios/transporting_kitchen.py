from typing import List

import rospy

from demos.pycram_multirobot_demo.setup.actions import actions
from demos.utils.enums import ROBOTS, ENVIRONMENTS
from demos.utils.launcher import launch_robot, launch_all_robots
from demos.utils.object_spawner import set_environment, create_robot
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import ObjectType, Arms, Grasp
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import NavigateAction, PickUpAction, PlaceAction
from pycram.designators.object_designator import BelieveObject
from pycram.process_module import simulated_robot
from pycram.world_concepts.world_object import Object


def transporting_kitchen(robots: List[ROBOTS], launch_robots=True):
    if launch_robots:
        launched_robots = launch_all_robots(robots=robots)

    robot_one = robots[0]
    robot_two = robots[1]

    pose_pr2 = Pose([1.5, 3, 0])
    pose_tiago = Pose([4, 3, 0])

    # Environment
    current_environment = set_environment(ENVIRONMENTS.KITCHEN)
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
    with simulated_robot(first_robot):
        actions(park=True)

        NavigateAction(target_locations=[Pose([1.3, 3, 0], orientation=[0, 0, 1, 0])]).resolve().perform()

        PickUpAction(object_designator_description=milk_BO,
                     arms=[Arms.LEFT],
                     grasps=[Grasp.FRONT]).resolve().perform()

        actions(park=True)

        NavigateAction(target_locations=[Pose([1.9, 3, 0], orientation=[0, 0, 0, 1])]).resolve().perform()

        PlaceAction(milk_BO, [Pose([2.75, 3, 1.02], orientation=[0, 0, 0, 1])], [Arms.LEFT]).resolve().perform()

        actions(park=True)

    rospy.sleep(3)
    print(f"{second_robot.name} actions")
    with simulated_robot(second_robot):
        actions(park=True)

        # NavigateAction(target_locations=[Pose([3.5, 3, 0], orientation=[0, 0, 1, 0])]).resolve().perform()
        NavigateAction(target_locations=[Pose([3.6, 3, 0], [0, 0, 1, 0])]).resolve().perform()

        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.75, 3, 1.02], orientation=[0, 0, 1, 0]),
                      color=Color(1, 0, 0, 1))

        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([3.3, 3.30, 0.62], orientation=[0, 0, 1, 0]),
                      color=Color(1, 0, 0, 1))

        second_robot.attach(milk)

        rospy.sleep(2)

        NavigateAction(target_locations=[Pose([4, 4, 0], [0, 0, 0, 1])]).resolve().perform()

        rospy.sleep(2)

        second_robot.detach(milk)

        milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([4.9, 3.8, 0.82], orientation=[0, 0, 1, 0]),
                      color=Color(1, 0, 0, 1))
