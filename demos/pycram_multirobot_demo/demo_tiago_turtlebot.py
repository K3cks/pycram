import rospy

from pycram.datastructures.enums import WorldMode
from pycram.datastructures.pose import Pose
from pycram.process_module import simulated_robot
from pycram.worlds.bullet_world import BulletWorld
from setup import ROBOTS, actions, create_robot

world = BulletWorld(WorldMode.GUI)

pose_pr2 = Pose([0, 1, 0])
pose_tiago = Pose([0, 3, 0])

print("Set first robot")
robot_turtle = create_robot(ROBOTS.TURTLE, pose=pose_pr2)
# current_environment = set_environment(ENVIRONMENTS.KITCHEN)
rospy.sleep(5)
print("Set second robot")
robot_tiago = create_robot(ROBOTS.TIAGO, pose=pose_tiago)
rospy.sleep(3)
print("pr2 actions")
with simulated_robot(robot_turtle):
    actions(navigate=True)

rospy.sleep(3)
print("tiago actions")
with simulated_robot(robot_tiago):
    actions(park=True, torso=True)

