from pycram import pose
from pycram.external_interfaces import robokudo
from pycram.process_module import real_robot, semi_real_robot
from pycram.designators.action_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

#broadcaster = TFBroadcaster()
#joint_publisher = JointStatePublisher("joint_states", 0.1)

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
object_mug = BelieveObject(names=["metalmug"])
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])


with real_robot:
    #todo we dont want to insert a object at all time
    object_desig = DetectAction(BelieveObject(types=[ObjectType.MILK])).resolve().perform()
    print("object desig is created")