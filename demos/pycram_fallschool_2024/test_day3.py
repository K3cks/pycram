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
from neem_interface_python import rosprolog_client as Prolog

world, apartment, milk_desig, robot_desig, apartment_desig, prolog_client = None, None, None, None, None, None


def init_prolog_client():
    global prolog_client
    prolog_client = Prolog.Prolog()
    retry = 7
    #    kb = KnowrobKnowledge()
    while (not prolog_client.prolog_client) and retry > 0:
        rospy.loginfo(f"[CRAM-KNOW] Waiting for knowrob connection... {retry} retries left.")
        prolog_client.connect()
        rospy.sleep(1)
        retry -= 1
    prolog_client.prolog_client.all_solutions(f"init_gpsr_2024.")
    rospy.loginfo("[CRAM-KNOW] Connected.")

# usage example: query("get_name_handle('Refrigerator', Result).")
# query("get_name_handle('Refrigerator', Result).").get('Result')
def query(query_string):
    global prolog_client
    result = prolog_client.ensure_once(query_string)
    return result


def setup_sim():
    global world, apartment, milk_desig, robot_desig, apartment_desig
    extension = ObjectDescription.get_file_extension()
    world = BulletWorld(WorldMode.DIRECT)
    viz = VizMarkerPublisher()
    tf = TFBroadcaster()

    pr2 = Object("pr2", ObjectType.ROBOT, "pr2.urdf")
    apartment = Object('apartment', ObjectType.ENVIRONMENT, f'apartment{extension}')
    # milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([0.7,2.5, 0.9]))
    # milk.color = Color(0, 0, 1, 1)
    milk_desig = BelieveObject(names=["milk"])
    robot_desig = BelieveObject(names=["pr2"])
    apartment_desig = BelieveObject(names=["apartment"])


def demo():
    global world, apartment, milk_desig, robot_desig, apartment_desig
    with simulated_robot:
        # ParkArmsAction([Arms.BOTH]).resolve().perform()
        Action(type='ParkArms', arms=[Arms.BOTH]).resolve().perform()
        # NavigateAction([Pose([0, 1, 0], [0, 0, 0, 1])]).resolve().perform()
        # go back to origin since poses are relative aparently
        Action(type='Navigate', target_locations=[Pose([0, 0, 0], [0, 0, 0, 1])]).resolve().perform()
        Action(type='Navigate', target_locations=[Pose([1.5, 2, 0], [0, 0, 0, 1])]).resolve().perform()
        # handle_desig = ObjectPart(names=["handle_cab3_door_top"], part_of=apartment_desig.resolve())
        # drawer_open_loc = AccessingLocation(handle_desig=handle_desig.resolve(),
        #                                     robot_desig=robot_desig.resolve()).resolve()
        # # NavigateAction([drawer_open_loc.pose]).resolve().perform()
        # Action(type='Navigate', target_locations=[drawer_open_loc.pose])
        # # OpenAction(object_designator_description=handle_desig, arms=[Arms.RIGHT]).resolve().perform()
        # Action(type='Open', object_designator_description=handle_desig, arms=[Arms.RIGHT]).resolve().perform()
        # # LookAtAction(targets=[milk_desig.resolve().pose]).resolve().perform()
        # Action(type='LookAt', targets=[milk_desig].resolve().pose).resolve().perform()
        #
        # # obj_desig = DetectAction(milk_desig).resolve().perform()
        # Action(type='Detect', object_designator_description=milk_desig).resolve().perform()
        # print("done")
