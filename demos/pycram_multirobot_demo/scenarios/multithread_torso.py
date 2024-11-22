import rospy

from demos.pycram_multirobot_demo.setup.actions import actions
from demos.utils.enums import ROBOTS
from demos.utils.launcher import launch_robot
from demos.utils.object_spawner import create_robot
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



def multithreaded_torsos(robot_one: ROBOTS, robot_two: ROBOTS, launch_robots=True):
    if launch_robots:
        first_robot_launch = launch_robot(robot_one, use_namespace=True)
        second_robot_launch = launch_robot(robot_two, use_namespace=True)

    pose_pr2 = Pose([0, 1, 0])
    pose_tiago = Pose([0, 3, 0])

    first_robot = create_robot(robot_one, pose=pose_pr2)
    second_robot = create_robot(robot_two, pose=pose_tiago)
    rospy.sleep(3)
    print(f"{first_robot.name} and {second_robot.name} actions")

    mtr = MultiThreadedRobot()
    process = mtr.start_process(robot_one_actions, (0.25, 0.0, first_robot, 5))
    process2 = mtr.start_process(robot_two_actions, (0.25, 0.0, second_robot, 10))
    mtr.end(process)
    mtr.end(process2)
