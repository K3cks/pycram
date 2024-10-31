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
from pycram.designators.designator_notation_interface import ActionDesignator as Action

extension = ObjectDescription.get_file_extension()
world = BulletWorld(WorldMode.DIRECT)
viz = VizMarkerPublisher()
tf = TFBroadcaster()

pr2 = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
apartment = Object('apartment', ObjectType.ENVIRONMENT, f'apartment{extension}')
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([0.4, 2.5, 1]))
# milk.color = Color(0, 0, 1, 1)
milk_desig = BelieveObject(names=["milk"])
robot_desig = BelieveObject(names=["pr2"])
apartment_desig = BelieveObject(names=["apartment"])

with simulated_robot:
    # ParkArmsAction([Arms.BOTH]).resolve().perform()
    Action(type='ParkArms', arms=[Arms.BOTH]).resolve().perform()
    # NavigateAction([Pose([0, 1, 0], [0, 0, 0, 1])]).resolve().perform()
    Action(type='Navigate', target_locations=[Pose([0, 1, 0], [0, 0, 0, 1])]).resolve().perform()
    handle_desig = ObjectPart(names=["handle_cab3_door_top"], part_of=apartment_desig.resolve())
    # drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
    #                                     robot_desig=robot_desig.resolve()).resolve()
    pose = Pose([1.3, 2.5, 0], [0, 0, 1, 0])
    drawer_open_loc = AccessingLocation.Location(pose, [Arms.LEFT])
    # NavigateAction([drawer_open_loc.pose]).resolve().perform()
    Action(type='Navigate', target_locations=[drawer_open_loc.pose]).resolve().perform()
    # OpenAction(object_designator_description=handle_desig, arms=[Arms.RIGHT]).resolve().perform()
    Action(type='Open', object_designator_description=handle_desig, arms=drawer_open_loc.arms).resolve().perform()
    # LookAtAction(targets=[milk_desig.resolve().pose]).resolve().perform()
    Action(type='LookAt', targets=[Pose([0.6, 2.5, 1])]).resolve().perform()

    # obj_desig = DetectAction(milk_desig).resolve().perform()
    Action(type='Detect', object_designator_description=milk_desig).resolve().perform()
    print("done")
