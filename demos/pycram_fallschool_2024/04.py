from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher, AxisMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color
from pycram.designators.object_designator import BelieveObject
extension = ObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.GUI)
viz = VizMarkerPublisher()
tf = TFBroadcaster()

pr2 = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
apartment = Object('apartment', ObjectType.ENVIRONMENT, f'apartment{extension}')
# milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([0.7,2.5, 0.9]))
#milk.color = Color(0, 0, 1, 1)
milk_desig = BelieveObject(names=["milk"])
robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])


with simulated_robot:
    ParkArmsAction([Arms.BOTH]).resolve().perform()
    pose = Pose([1.3, 2.5, 0], [0, 0, 1, 0])

    handle_desig = ObjectPart(names=["handle_cab3_door_top"], part_of=apartment_desig.resolve())
    closed_location, opened_location = AccessingLocation(handle_desig=handle_desig.resolve(),
                                                         robot_desig=robot_desig.resolve()).resolve()

    NavigateAction([closed_location.pose]).resolve().perform()

    OpenAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
               start_goal_location=[closed_location, opened_location]).resolve().perform()

