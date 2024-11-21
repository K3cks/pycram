import rospy

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import NavigateAction, ParkArmsAction, MoveTorsoAction
from demos.utils.enums import ROBOTS


def set_active_robot(robot: ROBOTS):
    robot_topic = 'multirobot_description/' + robot.name

    try:
        active_robot = rospy.get_param(robot_topic)
        rospy.set_param('robot_description', active_robot)
    except:
        rospy.logerr(f'No topic named {robot_topic} found')


def actions(park=False, torso=None, navigate=False):
    if park:
        rospy.sleep(2)
        ParkArmsAction([Arms.BOTH]).resolve().perform()

    if torso is not None:
        rospy.sleep(2)
        MoveTorsoAction([torso]).resolve().perform()

    if navigate:
        rospy.sleep(2)
        NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    print("done")
