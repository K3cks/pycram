import rospy

from demos.pycram_multirobot_demo.setup.actions import actions
from demos.utils.enums import ROBOTS, ENVIRONMENTS
from demos.utils.object_spawner import set_environment, create_robot
from pycram.datastructures.dataclasses import Color
from pycram.datastructures.enums import WorldMode, Grasp, Arms, ObjectType
from pycram.datastructures.pose import Pose
from pycram.designator import ObjectDesignatorDescription
from pycram.designators.action_designator import NavigateAction, MoveTorsoAction, PickUpAction
from pycram.process_module import real_robot, semi_real_robot, with_real_robot
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


def main():
    world = BulletWorld(WorldMode.DIRECT)
    viz = VizMarkerPublisher()
    # Environment
    current_environment = set_environment(ENVIRONMENTS.SUTURO)

    # Spawn Robots
    pose_hsrb = Pose([4.5, 5, 0])
    pose_turtle = Pose([4, 3, 0])

    hsrb = create_robot(ROBOTS.HSRB, pose=pose_hsrb, is_real=True)
    RobotStateUpdater("/tf", "/hsrb/joint_states", multirobot_name="hsrb")
    hsrb_desig = ObjectDesignatorDescription(names=["hsrb"])

    #milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]),
    #              color=Color(1, 0, 0, 1))

    test_obj = Object("test_obj", ObjectType.MILK, "milk.stl", pose=Pose([5.2, 5.2, 0.8]),
                  color=Color(1, 0, 0, 1))
    test_obj_desig = ObjectDesignatorDescription(names=["test_obj"])



    turtlebot = create_robot(ROBOTS.TURTLE, pose=pose_turtle)

    RobotStateUpdater("/tf", "/turtle/odom", multirobot_name="turtlebot")
    rospy.sleep(3)

    # HSR transporting on turtle
    print(f"{hsrb.name} actions")
    with real_robot(hsrb):
        MoveTorsoAction([0.15]).resolve().perform()
        rospy.sleep(3)
        MoveTorsoAction([0.05]).resolve().perform()
        rospy.sleep(3)
        NavigateAction(target_locations=[Pose([4.7, 5.18, 0])]).resolve().perform()
        rospy.sleep(3)
        print("Pickup")
        PickUpAction(test_obj_desig, [Arms.LEFT], [Grasp.FRONT]).resolve().perform()
        print("done")

    rospy.sleep(3)
    # Turtle moving towards goal table
    print(f"{turtlebot.name} actions")
    with real_robot(turtlebot):
        navigation_pose = Pose([0, 0, 0])
        NavigateAction(target_locations=[navigation_pose]).resolve().perform()

    with real_robot(hsrb):
        actions(park_arms=Arms.LEFT)


if __name__ == '__main__':
    main()
