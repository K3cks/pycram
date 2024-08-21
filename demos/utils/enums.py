from enum import Enum, auto


class ROBOTS(Enum):
    PANDA = 'panda'
    PR2 = 'pr2'
    HSRB = 'hsrb'
    TIAGO = 'tiago'
    BOXY = 'boxy'
    UR5 = 'ur5'
    TURTLE = 'turtle'
    TURTLEBOT3_WAFFLE_PI = 'turtlebot3_waffle_pi'
    DONBOT = 'donbot'
    STRETCH = 'stretch'

    ARMAR6 = 'armar6'


class ENVIRONMENTS(Enum):
    APARTMENT = auto()
    APARTMENT_SMALL = auto()
    KITCHEN = auto()
    SUTURO = auto()


class DEMOS(Enum):
    SIMPLE = auto()
    APARTMENT = auto()
    KITCHEN = auto()
    TRIPLE = auto()
