from typing import List

import rospy

from demos.pycram_multirobot_demo.demo_manager import robot_one
from demos.pycram_multirobot_demo.setup.actions import actions
from demos.utils.enums import ROBOTS, ENVIRONMENTS
from demos.utils.launcher import launch_robot, launch_all_robots
from demos.utils.object_spawner import create_robot, set_environment
from pycram.datastructures.pose import Pose
from pycram.multirobot.multi_threaded_robots import MultiThreadedRobot
from pycram.process_module import simulated_robot

def robot_one_actions(first_torso_value, second_torso_value, robot, iterations=10):
    with simulated_robot(robot):
        i = 0
        pose1 = Pose(position=[0, 1, 0])
        pose2 = Pose(position=[1, 0, 0])
        actions(park=True, used_robot=robot)

        while i < iterations:
            actions(navigate=pose1, used_robot=robot)

            actions(torso=first_torso_value, used_robot=robot)

            actions(torso=second_torso_value, used_robot=robot)

            actions(navigate=pose2, used_robot=robot)
            i += 1


def robot_two_actions(first_torso_value, second_torso_value, robot, iterations=10):
    with simulated_robot(robot):
        i = 0

        while i < iterations:
            actions(torso=first_torso_value, used_robot=robot)

            actions(torso=second_torso_value, used_robot=robot)
            i += 1


def robot_three_actions():
    pass


def robot_four_actions():
    pass


def party_apartment(robots: List[ROBOTS], launch_robots=True):
    if launch_robots:
        launched_robots = launch_all_robots(robots=robots)

    robot_one = robots[0]
    robot_two = robots[1]
    robot_three = robots[2]
    robot_four = robots[3]

    pose_pr2 = Pose([0, 1, 0])
    pose_tiago = Pose([0, 3, 0])
    pose_justin = Pose([3, 3, 0])
    pose_icub = Pose([3, 0, 0])

    current_environment = set_environment(ENVIRONMENTS.APARTMENT_SMALL)

    first_robot = create_robot(robot_one, pose=pose_pr2)
    second_robot = create_robot(robot_two, pose=pose_tiago)
    third_robot = create_robot(robot_three, pose=pose_justin)
    fourth_robot = create_robot(robot_four, pose=pose_icub)
    rospy.sleep(3)

    robot_one_actions(0.25, 0.0, first_robot)

    mtr = MultiThreadedRobot()
    process = mtr.start_process(robot_one_actions, (0.25, 0.0, first_robot, 5))
    process2 = mtr.start_process(robot_two_actions, (0.25, 0.0, second_robot, 10))
    process3 = mtr.start_process(robot_three_actions, (0.25, 0.0, third_robot, 10))
    process4 = mtr.start_process(robot_four_actions, (0.25, 0.0, fourth_robot, 10))
    mtr.end(process)
    mtr.end(process2)
