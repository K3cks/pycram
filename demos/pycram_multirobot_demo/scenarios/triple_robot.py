from typing import List

import rospy

from demos.pycram_multirobot_demo.setup.actions import actions
from demos.utils.enums import ROBOTS
from demos.utils.launcher import launch_robot, launch_all_robots
from demos.utils.object_spawner import create_robot
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot


def triple_robots(robots: List[ROBOTS], launch_robots=True):
    if launch_robots:
        launched_robots = launch_all_robots(robots=robots)

    robot_one = robots[0]
    robot_two = robots[1]
    robot_three = robots[2]

    pose_pr2 = Pose([0, 1, 0])
    pose_tiago = Pose([0, 3, 0])
    pose_third_robot = Pose([0, 5, 0])

    first_robot = create_robot(robot_one, pose=pose_pr2)
    second_robot = create_robot(robot_two, pose=pose_tiago)
    third_robot = create_robot(robot_three, pose=pose_third_robot)

    rospy.sleep(3)
    print(f"{first_robot.name} actions")
    with simulated_robot(first_robot):
        actions(park=True)

    rospy.sleep(3)
    print(f"{second_robot.name} actions")
    with simulated_robot(second_robot):
        actions(park=True, torso=0.25)

    with simulated_robot(third_robot):
        actions(park=True, torso=0.25)

    with simulated_robot(first_robot):
        actions(torso=0.25)
