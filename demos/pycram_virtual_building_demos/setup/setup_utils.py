import os
import base64
from IPython.display import display
import ipywidgets as widgets


def display_loading_gif_with_text():
    # Create a path for the GIF
    setup_path = os.path.dirname(os.path.realpath(__file__))
    gif_path = os.path.join(setup_path, 'imgs', 'loading.gif')

    # Encode the GIF in base64
    with open(gif_path, 'rb') as gif_file:
        b64_gif = base64.b64encode(gif_file.read()).decode('ascii')

    # Create an HTML widget to display the GIF
    gif_widget = widgets.HTML(f'<img src="data:image/gif;base64,{b64_gif}" />')

    # Create an HTML widget for larger text and light blue color
    text_widget = widgets.HTML('<span style="font-size: 40px; color: lightblue;">Loading, please wait...</span>')

    # Display the GIF and text next to each other
    display(widgets.HBox([gif_widget, text_widget]))

    return text_widget


# Function to update the text dynamically
def update_text(text_widget, new_text):
    text_widget.value = f'<span style="font-size: 40px; color: lightblue;">{new_text}</span>'


def get_robot_name(robot_name: str):
    """
    Retrieves the internal identifier for a specified robot name.

    This function maps a given `robot_name` to a predefined internal identifier.
    If `robot_name` does not match any known names, it returns the input unchanged.

    Args:
        robot_name (str): The name of the robot to map to its internal identifier.

    Returns:
        str: The internal identifier for the specified robot name, or the input
        `robot_name` if no predefined match is found.
    """
    robot_name_map = {
        'justin': 'rollin_justin',
        'donbot': 'iai_donbot',
        'tiago': 'tiago_dual',
        'armar': 'Armar6',
        'icub': 'iCub'
    }

    return robot_name_map.get(robot_name, robot_name)