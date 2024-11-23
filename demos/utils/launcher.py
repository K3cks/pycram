import time
from typing import List

import roslaunch
import rospy
import rospkg

from demos.utils.enums import ROBOTS


def launch_robot(robot: ROBOTS, use_namespace=False):
    folder = ''
    if use_namespace:
        folder = 'robots/'

    if robot == ROBOTS.PR2:
        executable = folder + 'pr2.launch'
    elif robot == ROBOTS.TIAGO:
        executable = folder + 'tiago.launch'
    elif robot == ROBOTS.DONBOT:
        executable = folder + 'donbot.launch'
    elif robot == ROBOTS.STRETCH:
        executable = folder + 'stretch.launch'
    elif robot == ROBOTS.ARMAR6:
        executable = folder + 'armar6.launch'
    elif robot == ROBOTS.TURTLE:
        executable = folder + 'turtlebot.launch'
    elif robot == ROBOTS.ICUB:
        executable = folder + 'icub.launch'
    elif robot == ROBOTS.JUSTIN:
        executable = folder + 'rollin_justin.launch'
    else:
        raise Exception(f'Robot {robot} is not supported')

    launch_file(executable)


def launch_file(file, package='pycram', launch_folder='/launch/'):
    """
    General method to start a specified launch file with given parameters.
    Default location for launch files here is in the folder 'launch' inside the pycram package

    :param file: File name of the launch file
    :param package: Name of the package
    :param launch_folder: Location of the launch file inside the package
    """

    rospath = rospkg.RosPack()

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [rospath.get_path(package) + launch_folder + file])
    launch.start()

    rospy.loginfo(f'{file} started')

    # Wait for ik server to launch
    time.sleep(2)


def launch_all_robots(robots: List[ROBOTS]):
    launched_robots = []
    for robot in robots:
        launched_robots.append(launch_robot(robot, use_namespace=True))
    return launched_robots
