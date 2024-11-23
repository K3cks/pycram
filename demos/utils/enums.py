from enum import Enum, auto


class ROBOTS(Enum):
    PANDA = 'panda'
    PR2 = 'pr2'
    HSRB = 'hsrb'
    TIAGO = 'tiago_dual'
    BOXY = 'boxy'
    UR5 = 'ur5'
    TURTLE = 'turtlebot'
    DONBOT = 'donbot'
    STRETCH = 'stretch'
    ARMAR6 = 'armar6'
    ICUB = 'iCub'
    JUSTIN = 'rollin_justin'


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
    THREADED_TEST = auto()
    PARTY = auto()
