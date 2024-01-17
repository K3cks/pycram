from pycram.process_module import real_robot
from pycram.designators.action_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
#07.12.23 worked with....

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])


# Initialize objects in BulletWorld
# TODO: Warum ist die Orientierung vom falschen Datentyp? Kann ich das ignorieren?
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])


# Giskard initialisieren und syncen
giskardpy.init_giskard_interface()
#giskardpy.sync_worlds()

#RobotStateUpdater("/tf", "/joint_states")

with real_robot:
    NavigateAction(target_locations=[Pose([3.7, 1.4, 0])]).resolve().perform()
