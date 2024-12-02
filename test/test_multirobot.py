import time
import unittest

from urdf_parser_py.urdf import URDF

import pycram.tasktree
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.datastructures.pose import Pose
from pycram.multirobot import RobotManager
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.ontology.ontology import OntologyManager, SOMA_ONTOLOGY_IRI
from pycram.process_module import ProcessModule
from pycram.robot_description import RobotDescription
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld


class MultiRobotTestCase(unittest.TestCase):
    world: BulletWorld
    viz_marker_publisher: VizMarkerPublisher
    extension: str = ObjectDescription.get_file_extension()

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld(mode=WorldMode.DIRECT)

        pose_pr2 = Pose([0, 1, 0])
        cls.robot_pr2 = Object("pr2", ObjectType.ROBOT, 'pr2' + cls.extension, pose=pose_pr2)

        pose_tiago = Pose([0, 3, 0])
        cls.robot_tiago = Object("tiago_dual", ObjectType.ROBOT, "tiago_dual" + cls.extension, pose=pose_tiago)

        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([0.5, 3, 1.02], orientation=[0, 0, 1, 0]))

        ProcessModule.execution_delay = False
        cls.viz_marker_publisher = VizMarkerPublisher()
        OntologyManager(SOMA_ONTOLOGY_IRI)

    def setUp(self):
        self.world.reset_world()

    def tearDown(self):
        pycram.tasktree.reset_tree()
        time.sleep(0.05)
        self.world.reset_world()

    @classmethod
    def tearDownClass(cls):
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()

    def check_robot(self, name, base_link, torso_link, torso_joint, number_of_links, number_of_joints):
        self.assertEqual(RobotDescription.current_robot_description.name, name)
        self.assertEqual(RobotDescription.current_robot_description.base_link, base_link)
        self.assertEqual(RobotDescription.current_robot_description.torso_link, torso_link)
        self.assertEqual(RobotDescription.current_robot_description.torso_joint, torso_joint)
        self.assertTrue(type(RobotDescription.current_robot_description.urdf_object) is URDF)
        self.assertEqual(len(RobotDescription.current_robot_description.links), number_of_links)
        self.assertEqual(len(RobotDescription.current_robot_description.joints), number_of_joints)

    def check_current_robot(self, name):
        if name == "pr2":
            self.check_robot(name="pr2", base_link="base_link", torso_link="torso_lift_link",
                             torso_joint="torso_lift_joint", number_of_links=88, number_of_joints=87)
        elif name == "tiago_dual":
            self.check_robot(name="tiago_dual", base_link="base_link", torso_link="torso_lift_link",
                             torso_joint="torso_lift_joint", number_of_links=69, number_of_joints=68)


class TestMultiRobot(MultiRobotTestCase):

    def test_load_multiple_robots(self):
        RobotManager.set_active_robot(self.robot_pr2.name)

        self.check_current_robot(self.robot_pr2.name)

        RobotManager.set_active_robot(self.robot_tiago.name)

        self.check_current_robot(self.robot_tiago.name)

    def test_transport_object(self):
        """
        Transport an object using a sequence of actions for two robots.
        """
        pass

    def test_object_blocked(self):
        """
        Test that an Action which tries to use a blocked object will be canceled
        """

        RobotManager.block_object(object_desig=self.milk, robot_name=self.robot_pr2.name)

        self.assertTrue(RobotManager.is_object_blocked(self.milk))

        RobotManager.release_object(object_desig=self.milk)

    def test_object_released(self):
        """
        Test that a blocked object is correctly released when an action is done
        """

        RobotManager.block_object(object_desig=self.milk, robot_name=self.robot_pr2.name)

        RobotManager.release_object(object_desig=self.milk)

        self.assertTrue(not RobotManager.is_object_blocked(self.milk))
