from pycram.external_interfaces.ik import request_ik
from pycram.plan_failures import IKError
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher, AxisMarkerPublisher, CostmapPublisher
from pycram.utils import _apply_ik
from pycram.worlds.bullet_world import BulletWorld
from pycram.designators.action_designator import *
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.datastructures.enums import ObjectType, WorldMode, TorsoState
from pycram.datastructures.pose import Pose, Transform
from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object
from pycram.datastructures.dataclasses import Color

extension = ObjectDescription.get_file_extension()

world = BulletWorld(WorldMode.DIRECT)
world.allow_publish_debug_poses = True
viz = VizMarkerPublisher()
tf = TFBroadcaster()

robot_name = "pr2"
robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))

apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment-small{extension}")

robot_desig = BelieveObject(names=[robot_name])
apartment_desig = BelieveObject(names=["apartment"])
#


milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([0.5, 2.5, 1]))
milk.color = Color(0, 0, 1, 1)
milk_desig = BelieveObject(names=["milk"])

with simulated_robot:
    poseHard = Pose([1.3, 2.7, 0], [0, 0, 1, 0])
    NavigateAction([poseHard]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()
    handle_desig = ObjectPart(names=["handle_cab3_door_top"], part_of=apartment_desig.resolve())
    closed_location, opened_location = AccessingLocation(handle_desig=handle_desig.resolve(),
                                                         robot_desig=robot_desig.resolve()).resolve()
    OpenAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
               start_goal_location=[closed_location, opened_location]).resolve().perform()

    LookAtAction(targets=[milk_desig.resolve().pose]).resolve().perform()
    obj_desig = DetectAction(milk_desig).resolve().perform()
    milk_target_pose = Pose([4.8, 3.45, 0.8])
    TransportAction(obj_desig, [Arms.LEFT], [milk_target_pose]).resolve().perform()



    CloseAction(object_designator_description=handle_desig, arms=[closed_location.arms[0]],
                        start_goal_location=[opened_location, closed_location]).resolve().perform()

