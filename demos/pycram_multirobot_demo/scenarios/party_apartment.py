from demos.utils.enums import ROBOTS, ENVIRONMENTS
from demos.utils.launcher import launch_all_robots
from demos.utils.object_spawner import create_robot, set_environment
from pycram.designators.action_designator import *
from pycram.multirobot.multi_threaded_robots import MultiThreadedRobot
from pycram.process_module import simulated_robot


def robot_one_actions(robot, iterations=1):
    i = 0
    delay = 2
    while i < iterations:
        with simulated_robot(robot):
            ParkArmsAction([Arms.BOTH], used_robot=robot).resolve().perform()
            rospy.sleep(delay)
            PartyAction(arms=[Arms.BOTH], used_robot=robot).resolve().perform()
            rospy.sleep(delay)
            i = i + 1


def robot_two_actions(robot, iterations=1):
    i = 0
    delay = 2
    while i < iterations:
        with simulated_robot(robot):
            ParkArmsAction([Arms.BOTH], used_robot=robot).resolve().perform()
            rospy.sleep(delay)
            PartyAction(arms=[Arms.BOTH], used_robot=robot).resolve().perform()
            rospy.sleep(delay)
            i = i + 1


def robot_three_actions(robot, iterations=1):
    i = 0
    delay = 2
    while i < iterations:
        with simulated_robot(robot):
            ParkArmsAction([Arms.BOTH], used_robot=robot).resolve().perform()
            rospy.sleep(delay)
            PartyAction(arms=[Arms.BOTH], used_robot=robot).resolve().perform()
            rospy.sleep(delay)
            i = i + 1


def robot_four_actions(robot, iterations=1):
    i = 0
    delay = 2
    while i < iterations:
        with simulated_robot(robot):
            ParkArmsAction([Arms.BOTH], used_robot=robot).resolve().perform()
            rospy.sleep(delay)
            PartyAction(arms=[Arms.BOTH], used_robot=robot).resolve().perform()
            rospy.sleep(delay)
            i = i + 1


def party_apartment(robots: List[ROBOTS], launch_robots=True):
    if launch_robots:
        launched_robots = launch_all_robots(robots=robots)

    robot_one = robots[0]
    robot_two = robots[1]
    robot_three = robots[2]
    robot_four = robots[3]

    pose_pr2 = Pose([1.5, 2.5, 0])
    pose_tiago = Pose([4.5, 5.3, 0], [0, 0, -0.2, 0.3])
    pose_justin = Pose([5, 2, 0], [0, 0, 0.2, 0.3])
    pose_icub = Pose([4, 3.8, 0], [0, 0, 0, 1])

    current_environment = set_environment(ENVIRONMENTS.APARTMENT_SMALL)

    first_robot = create_robot(robot_one, pose=pose_pr2)
    second_robot = create_robot(robot_two, pose=pose_tiago)
    third_robot = create_robot(robot_three, pose=pose_justin)
    fourth_robot = create_robot(robot_four, pose=pose_icub)
    rospy.sleep(3)

    mtr = MultiThreadedRobot()
    process = mtr.start_process(robot_one_actions, (first_robot, 3))
    process2 = mtr.start_process(robot_two_actions, (second_robot, 5))
    process3 = mtr.start_process(robot_three_actions, (third_robot, 7))
    process4 = mtr.start_process(robot_four_actions, (fourth_robot, 9))
    mtr.wait_for_all_processes()
