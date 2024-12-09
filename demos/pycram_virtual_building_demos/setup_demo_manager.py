import sys

from IPython.core.display_functions import clear_output

from demos.pycram_virtual_building_demos.src.cleanup_demo import cleanup_demo
from demos.pycram_virtual_building_demos.src.follow_demo import follow_simple_example
from demos.pycram_virtual_building_demos.src.generlized_actions_demo import start_generalized_demo
from demos.pycram_virtual_building_demos.src.transport_demo import transporting_demo

# sys.path.insert(0, '/home/vee/robocup_workspaces/pycram_ws/src/pycram')
# sys.path.insert(0, '/home/jovyan/workspace/ros/src/pycram')
# sys.path.insert(0, '/home/me/IAI_work/binder_ws/src/pycram')

from demos.pycram_virtual_building_demos.setup.setup_utils import display_loading_gif_with_text, update_text, \
    get_robot_name

from demos.pycram_virtual_building_demos.src.simple_examples import navigate_simple_example
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.datastructures.enums import WorldMode
from pycram.designators.action_designator import *
from pycram.designators.object_designator import *
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.ros.viz_marker_publisher import VizMarkerPublisher, VizMarkerRobotPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

output = None


def start_demo():
    # get params
    environment_param = rospy.get_param('/nbparam_environments')
    robot_param = rospy.get_param('/nbparam_robots')
    task_param = rospy.get_param('/nbparam_tasks')

    robot_name = get_robot_name(robot_param)

    extension = ObjectDescription.get_file_extension()
    # text widget for the virtual building
    text_widget = display_loading_gif_with_text()
    update_text(text_widget, 'Loading process~ Please wait...')
    world = BulletWorld(WorldMode.DIRECT)

    # Set this to True to publish costmaps and axis marker during the demo. May slow down the simulation.
    world.allow_publish_debug_poses = False

    VizMarkerPublisher()
    robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))
    apartment = Object(environment_param, ObjectType.ENVIRONMENT, f"{environment_param}{extension}")
    if not robot.name == "Armar6":
        TFBroadcaster()

    clear_output(wait=True)

    update_text(text_widget, 'Executing Demo: ' + task_param)

    demo_selecting(environment_param, robot_name, task_param)

    update_text(text_widget, 'Done with: ' + task_param)


def start_demo_local():
    # get params
    environment_param = 'apartment'
    robot_param = 'stretch'
    task_param = 'transporting'

    robot_name = get_robot_name(robot_param)

    extension = ObjectDescription.get_file_extension()

    world = BulletWorld(WorldMode.DIRECT)
    VizMarkerPublisher(interval=0.4)
    VizMarkerRobotPublisher(interval=0.2)
    robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}{extension}", pose=Pose([1, 2, 0]))
    apartment = Object(environment_param, ObjectType.ENVIRONMENT, f"{environment_param}-small{extension}")
    TFBroadcaster()


    demo_selecting(environment_param, robot_name, task_param)
    extension = ObjectDescription.get_file_extension()


def demo_selecting(apartment, robot, task_param):
    # if task_param == "navigate":
    #     navigate_simple_example()
    if task_param == "follow":
        follow_simple_example(robot)
    elif task_param == "transporting" or task_param == "navigate":
        specialized_task = None
        # specialized_task = rospy.get_param('/nbparam_specialized_task')
        if specialized_task == "clean":
            cleanup_demo(apartment, robot)
        else:
            transporting_demo(apartment, robot)
    elif task_param in ["cutting", "mixing", "pouring"]:
        # object_target = rospy.get_param('/nbparam_object')
        # object_tool = rospy.get_param('/nbparam_object_tool')
        if task_param == "mixing":
            object_target = "big-bowl"
            object_tool = "whisk"
        elif task_param == "pouring":
            object_target = "bowl"
            object_tool = "jeroen_cup"
        else:
            object_target = "banana"
            object_tool = "butter_knife"
        specialized_task = rospy.get_param('/nbparam_specialized_task')
        start_generalized_demo(task_param, object_tool, object_target, specialized_task)

# start_demo_local()
