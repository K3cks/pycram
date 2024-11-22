import rospy

from demos.pycram_multirobot_demo.setup.actions import actions
from demos.utils.enums import ROBOTS
from demos.utils.launcher import launch_robot
from demos.utils.object_spawner import create_robot
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot


def move_and_park(robot_one: ROBOTS, robot_two: ROBOTS, launch_robots=True):
    if launch_robots:
        first_robot_launch = launch_robot(robot_one, use_namespace=True)
        second_robot_launch = launch_robot(robot_two, use_namespace=True)

    pose_pr2 = Pose([0, 1, 0])
    pose_tiago = Pose([0, 3, 0])

    first_robot = create_robot(robot_one, pose=pose_pr2)
    second_robot = create_robot(robot_two, pose=pose_tiago)
    rospy.sleep(3)
    print(f"{first_robot.name} actions")
    with simulated_robot(first_robot):
        goal_pose = Pose([1, 1, 0])
        actions(park=True, navigate=goal_pose)

    rospy.sleep(3)
    print(f"{second_robot.name} actions")
    with simulated_robot(second_robot):
        actions(park=True, torso=0.25)

    with simulated_robot(first_robot):
        actions(torso=0.25)
