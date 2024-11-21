from IPython.core.display_functions import display
from ipywidgets import HTML

from demos.pycram_multirobot_demo.scenarios.move_and_park import move_and_park
from demos.pycram_multirobot_demo.scenarios.multithread_torso import multithreaded_torsos
from demos.pycram_multirobot_demo.scenarios.transporting_apartment import transporting_apartment
from demos.pycram_multirobot_demo.scenarios.transporting_kitchen import transporting_kitchen
from demos.pycram_multirobot_demo.scenarios.triple_robot import triple_robots
from demos.utils.enums import DEMOS, ROBOTS
from pycram.datastructures.enums import WorldMode
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld


def multirobot_demo(robot_one: ROBOTS = ROBOTS.PR2, robot_two: ROBOTS = ROBOTS.TIAGO,
                    demo=DEMOS.APARTMENT, mode=WorldMode.GUI):
    world = BulletWorld(mode)
    viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

    if demo == DEMOS.SIMPLE:
        move_and_park(robot_one=robot_one, robot_two=robot_two)
    elif demo == DEMOS.APARTMENT:
        transporting_apartment(robot_one=robot_one, robot_two=robot_two)
    elif demo == DEMOS.KITCHEN:
        transporting_kitchen(robot_one=robot_one, robot_two=robot_two)
    elif demo == DEMOS.TRIPLE:
        triple_robots(robot_one=robot_one, robot_two=robot_two, robot_three=ROBOTS.ARMAR6)
    elif demo == DEMOS.THREADED:
        multithreaded_torsos(robot_one=robot_one, robot_two=robot_two)


def multirobot_demo_binder(robot_one, robot_two, environment):
    display(HTML('<img src="https://i.gifer.com/XVo6.gif" alt="Hourglass animation" width="50">'))
    multirobot_demo(robot_one=robot_one, robot_two=robot_two, demo=environment, mode=WorldMode.GUI)


if __name__ == '__main__':
    r1 = ROBOTS.PR2
    r2 = ROBOTS.TIAGO
    demo = DEMOS.THREADED
    mode = WorldMode.DIRECT

    multirobot_demo(robot_one=r1, robot_two=r2, demo=demo, mode=mode)
