from typing import Optional

import rospy

from pycram.datastructures.enums import Arms
from pycram.datastructures.pose import Pose
from pycram.designators.action_designator import NavigateAction, ParkArmsAction, MoveTorsoAction
from pycram.world_concepts.world_object import Object


def actions(park_arms: Optional[Arms] = None,
            torso: Optional[float] = None,
            navigate: Optional[Pose] = None,
            used_robot: Optional[Object] = None,
            use_logging: bool = True):
    if park_arms is not None:
        rospy.sleep(2)
        ParkArmsAction([park_arms], used_robot=used_robot).resolve().perform()

    if torso is not None:
        rospy.sleep(2)
        MoveTorsoAction([torso], used_robot=used_robot).resolve().perform()

    if navigate is not None:
        rospy.sleep(2)
        NavigateAction(target_locations=[navigate], used_robot=used_robot).resolve().perform()

    if use_logging:
        rospy.loginfo(f"Done")
