from pycram.external_interfaces.ik import request_ik
from pycram.plan_failures import IKError
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
viz = VizMarkerPublisher()
robot_name = "pr2"
robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))

apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment-small{extension}")

milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]),
              color=Color(1, 0, 0, 1))
cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                pose=Pose([2.5, 2.5, 1.05]), color=Color(0, 1, 0, 1))
bowl = Object("bowl", ObjectType.BOWL, "bowl.stl", pose=Pose([2.4, 2.2, 0.98]),
              color=Color(1, 1, 0, 1))
spoon = Object("spoon", ObjectType.SPOON, "spoon.stl", pose=Pose([2.4, 2.2, 0.85]),
               color=Color(0, 0, 1, 1))
apartment.attach(spoon, 'cabinet10_drawer_top')

pick_pose = Pose([2.7, 2.15, 1])

robot_desig = BelieveObject(names=[robot_name])
apartment_desig = BelieveObject(names=["apartment"])


@with_simulated_robot
def move_and_detect(obj_type):
    NavigateAction(target_locations=[Pose([1.7, 2, 0])]).resolve().perform()

    LookAtAction(targets=[pick_pose]).resolve().perform()

    object_desig = DetectAction(BelieveObject(types=[obj_type])).resolve().perform()

    return object_desig


with simulated_robot:
    NavigateAction([Pose([0, 0, 0])]).resolve().perform()

    ParkArmsAction([Arms.BOTH]).resolve().perform()

    MoveTorsoAction([TorsoState.HIGH]).resolve().perform()

    milk_desig = move_and_detect(ObjectType.MILK)

    TransportAction(milk_desig, [Arms.LEFT], [Pose([4.8, 3.55, 0.8])]).resolve().perform()

    cereal_desig = move_and_detect(ObjectType.BREAKFAST_CEREAL)

    TransportAction(cereal_desig, [Arms.LEFT], [Pose([5.2, 3.4, 0.8], [0, 0, 1, 1])]).resolve().perform()

    bowl_desig = move_and_detect(ObjectType.BOWL)

    TransportAction(bowl_desig, [Arms.LEFT], [Pose([5, 3.3, 0.8], [0, 0, 1, 1])]).resolve().perform()

    # Finding and navigating to the drawer holding the spoon
    handle_desig = ObjectPart(names=["handle_cab10_t"], part_of=apartment_desig.resolve())

    if robot.name == "rollin_justin":
        pose = Pose([1.4, 1.6, 0], [0, 0, 0.2040033016133158, 0.9789702002261697])
        drawer_open_loc = AccessingLocation.Location(pose, [Arms.LEFT])
    else:
        drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
                                            robot_desig=robot_desig.resolve()).resolve()

    NavigateAction([drawer_open_loc.pose]).resolve().perform()

    OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
    spoon.detach(apartment)

    # Detect and pickup the spoon
    LookAtAction([apartment.get_link_pose("handle_cab10_t")]).resolve().perform()
    ParkArmsAction([Arms.BOTH]).resolve().perform()

    spoon_desig = DetectAction(BelieveObject(types=[ObjectType.SPOON])).resolve().perform()

    if robot.name == "iai_donbot":
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        PickUpAction(spoon_desig, [Arms.LEFT], [Grasp.TOP]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        # Find a pose to place the spoon, move and then place it
        spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])
        placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve()).resolve()

        NavigateAction([placing_loc.pose]).resolve().perform()

        PlaceAction(spoon_desig, [spoon_target_pose], [Arms.LEFT]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        NavigateAction([drawer_open_loc.pose]).resolve().perform()

        CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

    else:
        if robot.name == "tiago_dual":
            NavigateAction([Pose([1.45, 2.7, 0], [0, 0, 0, 1])]).resolve().perform()

        pickup_arm = Arms.LEFT if drawer_open_loc.arms[0] == Arms.RIGHT else Arms.RIGHT
        ParkArmsAction([Arms.BOTH]).resolve().perform()
        PickUpAction(spoon_desig, [pickup_arm], [Grasp.TOP]).resolve().perform()

        ParkArmsAction([Arms.LEFT if pickup_arm == Arms.LEFT else Arms.RIGHT]).resolve().perform()

        NavigateAction([drawer_open_loc.pose]).resolve().perform()

        CloseAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()

        MoveTorsoAction([TorsoState.MID]).resolve().perform()

        # Find a pose to place the spoon, move and then place it
        spoon_target_pose = Pose([4.85, 3.3, 0.8], [0, 0, 1, 1])
        placing_loc = CostmapLocation(target=spoon_target_pose, reachable_for=robot_desig.resolve(),
                                      reachable_arm=pickup_arm, used_grasps=[Grasp.TOP]).resolve()

        NavigateAction([placing_loc.pose]).resolve().perform()

        PlaceAction(spoon_desig, [spoon_target_pose], [pickup_arm]).resolve().perform()

        ParkArmsAction([Arms.BOTH]).resolve().perform()
