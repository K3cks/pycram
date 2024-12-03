from IPython.core.display_functions import display
from ipywidgets import widgets

from demos.pycram_hsrb_follow.variable_handler import VariableHandler
from pycram.datastructures.enums import WorldMode
from pycram.designators.action_designator import *
from pycram.process_module import simulated_robot
from pycram.ros.tf_broadcaster import TFBroadcaster
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


def start_demo():
    output = widgets.Output()

    environment_param = 'apartment'
    robot_name = 'hsrb'

    # World
    world = BulletWorld(WorldMode.DIRECT)
    VizMarkerPublisher(ignore_robots=False)
    tf = TFBroadcaster()
    vh = VariableHandler()

    # Objects
    robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}.urdf", pose=Pose([1, 2, 0]))
    human = Object("human", ObjectType.MILK, "human.stl", pose=Pose([2, 0, 0], vh.ori_s))
    apartment = Object(environment_param, ObjectType.ENVIRONMENT, f"{environment_param}-small.urdf")

    vh.set_robot(robot)
    vh.set_human(human)
    vh.set_output(output)

    def move_up(_):
        vh.move_forward()
    def move_down(_):
        vh.move_back()
    def move_left(_):
        vh.move_left()
    def move_right(_):
        vh.move_right()
    def go_back(_):
        vh.go_back()
    def reset_demo(_):
        vh.reset_demo()


    # Place robot behind human
    with simulated_robot:
        pose = Pose([1, 0, 0], vh.ori_a)
        NavigateAction([pose]).resolve().perform()

    up_button = widgets.Button(description="Forward", button_style="success")
    down_button = widgets.Button(description="Back", button_style="success")
    left_button = widgets.Button(description="Left", button_style="success")
    right_button = widgets.Button(description="Right", button_style="success")
    back_button = widgets.Button(description="Go back", button_style="warning")
    reset_button = widgets.Button(description="Reset", button_style="danger")

    # Attach event handlers
    up_button.on_click(move_up)
    down_button.on_click(move_down)
    left_button.on_click(move_left)
    right_button.on_click(move_right)
    back_button.on_click(go_back)
    reset_button.on_click(reset_demo)

    # Arrange buttons in a "WASD" layout with offset
    buttons = widgets.VBox([
        widgets.HBox([widgets.Label(" ")] * 38 + [up_button] + [widgets.Label(" ")]),  # W (Up) shifted right
        widgets.HBox([left_button, down_button, right_button]),  # A (Left), S (Down), D (Right)
        widgets.HBox([back_button] + [widgets.Label(" ")] * 38 + [reset_button])  # Reset button in the center
    ])

    # Display everything
    display(buttons, output)

    # Demo is now ready using the buttons in jupyter notebook file
