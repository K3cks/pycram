import logging
import time
from typing import List

import roslaunch
import rospy
import rospkg


def launch_pr2():
    """
    Method to launch PR2 on the parameter server and start the ik service
    """
    # name = 'pr2'
    # urdf = 'pr2.urdf'
    executable = 'pr2_standalone.launch'
    launch_robot(executable)


def launch_hsrb():
    # name = 'hsrb'
    # urdf = 'hsrb.urdf'
    executable = 'hsrb_standalone.launch'
    launch_robot(executable)


def launch_stretch():
    # name = 'stretch'
    # urdf = 'stretch_description.urdf'
    executable = 'ik_and_description.launch'
    args = ["robot:=stretch"]
    launch_robot(executable, args=args)


def launch_tiago():
    # name = 'tiago_dual'
    # urdf = 'tiago_dual.urdf'
    executable = 'ik_and_description.launch'
    args = ["robot:=tiago_dual"]
    launch_robot(executable, args=args)


def launch_justin():
    # name = 'rollin_justin'
    # urdf = 'rollin_justin.urdf'
    executable = 'ik_and_description.launch'
    args = ["robot:=justin"]
    launch_robot(executable, args=args)

def launch_donbot():
    # name = 'iai_donbot'
    # urdf = 'iai_donbot.urdf'
    executable = 'ik_and_description.launch'
    args = ["robot:=donbot"]
    launch_robot(executable, args=args)

def launch_armar():
    # name = 'Armar6'
    # urdf = 'Armar6.urdf'
    executable = 'ik_and_description.launch'
    args = ["robot:=armar"]
    launch_robot(executable, args=args)

def launch_icub():
    # name = 'iCub'
    # urdf = 'iCub.urdf'
    executable = 'ik_and_description.launch'
    args = ["robot:=icub"]
    launch_robot(executable, args=args)

def launch_robot(launch_file, package='pycram', launch_folder='/launch/', args: List[str] = None):
    """
    Starts a specified launch file with given parameters.

    The default location for launch files is the 'launch' folder inside the pycram package.

    Args:
        launch_file (str): File name of the launch file.
        package (str): Name of the package.
        launch_folder (str): Location of the launch file inside the package.
        args (list): List of arguments to pass to the launch file.

    Returns:
        None
    """
    rospath = rospkg.RosPack()

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_args = rospath.get_path(package) + launch_folder + launch_file

    if args is None:
        args = [""]

    args.insert(0, launch_args)
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(args)[0], args[1:])]
    print(roslaunch_file)
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    launch.start()

    rospy.loginfo(f'{launch_file} started')
    time.sleep(2)
