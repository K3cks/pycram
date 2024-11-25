from typing import List

from IPython.core.display_functions import display
from ipywidgets import HTML

from demos.pycram_multirobot_demo.scenarios.move_and_park import move_and_park
from demos.pycram_multirobot_demo.scenarios.multithread_testing import multithreaded_testing
from demos.pycram_multirobot_demo.scenarios.party_apartment import party_apartment
from demos.pycram_multirobot_demo.scenarios.transporting_apartment import transporting_apartment
from demos.pycram_multirobot_demo.scenarios.transporting_kitchen import transporting_kitchen
from demos.pycram_multirobot_demo.scenarios.triple_robot import triple_robots
from demos.utils.enums import DEMOS, ROBOTS
from pycram.datastructures.enums import WorldMode
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.worlds.bullet_world import BulletWorld


def multirobot_demo(robots: List[ROBOTS], demo=DEMOS.APARTMENT, mode=WorldMode.GUI):
    world = BulletWorld(mode)
    viz = VizMarkerPublisher() if mode == WorldMode.DIRECT else None

    if demo == DEMOS.SIMPLE:
        move_and_park(robots=robots)
    elif demo == DEMOS.APARTMENT:
        transporting_apartment(robots=robots)
    elif demo == DEMOS.KITCHEN:
        transporting_kitchen(robots=robots)
    elif demo == DEMOS.TRIPLE:
        triple_robots(robots=robots)
    elif demo == DEMOS.THREADED_TEST:
        multithreaded_testing(robots=robots)
    elif demo == DEMOS.PARTY:
        party_apartment(robots=robots)


def multirobot_demo_binder(robots, environment):
    display(HTML('<img src="https://i.gifer.com/XVo6.gif" alt="Hourglass animation" width="50">'))
    multirobot_demo(robots=robots, demo=environment, mode=WorldMode.GUI)


if __name__ == '__main__':
    robots = [
        ROBOTS.PR2,
        ROBOTS.TIAGO,
        ROBOTS.JUSTIN,
        ROBOTS.ICUB
    ]

    demo = DEMOS.PARTY
    mode = WorldMode.DIRECT

    multirobot_demo(robots=robots, demo=demo, mode=mode)
