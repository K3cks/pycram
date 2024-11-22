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


def actions(park=False, torso=None, navigate=None, used_robot=None):
    if park:
        rospy.sleep(2)
        ParkArmsAction([Arms.BOTH], used_robot=used_robot).resolve().perform()

    if torso is not None:
        rospy.sleep(2)
        MoveTorsoAction([torso], used_robot=used_robot).resolve().perform()

    if navigate is not None:
        rospy.sleep(2)
        NavigateAction(target_locations=[navigate], used_robot=used_robot).resolve().perform()

    print("done")
