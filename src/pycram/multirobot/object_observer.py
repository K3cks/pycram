from typing import Dict, Union, List

import rospy


class ObjectObserver:
    """
    Class to observe the state of objects that are currently in use and shouldn't be accessed by another robot
    """

    blocked_objects: Dict[int, Dict[str, Union[str, List[str], int]]] = {}
    """
    Variable that stores the array of objects
    """

    def __init__(self):
        """
        Currently does nothing
        """

    def add_blocker(self, obj, amount_of_collaborating_robots = 1):
        """
        Adds an object to the observer list
        """

        object_details = {"robots": [],
                          "name": obj.name,
                          "robots_needed": amount_of_collaborating_robots}

        self.blocked_objects[obj.id] = object_details

    def remove_blocker(self, obj):
        """
        Remove an object from the observer list
        """
        self.blocked_objects.pop(obj.id)

    def assign_robot(self, obj, robot_name: str):
        """
        Assign a robot to a task that handles a given object
        """
        if not self.is_object_in_use(obj):
            rospy.loginfo(f"Object {obj} not in use")
            return

        if self.can_robot_do_collab_action(self.blocked_objects[obj.id]):
            self.blocked_objects[obj.id]["robots"].append(robot_name)
            rospy.loginfo(f"Robot {robot_name} added for collaboration on {obj.name}")


    def release_robot(self, obj, robot_name):
        """
        Release a robot from a task that handles a given object
        """
        if not self.is_object_in_use(obj):
            return False

        if robot_name not in self.blocked_objects[obj.id]["robots"]:
            rospy.logwarn(f"Robot {robot_name} not added for a task on {obj.name}")
            return False

        self.blocked_objects[obj.id]["robots"].remove(robot_name)

        if len(self.blocked_objects[obj.id]["robots"]) == 0:
            self.remove_blocker(obj)
            rospy.loginfo(f"Deleted blocker for object {obj.name}")

    def is_object_in_use(self, obj):
        all_ids = list(self.blocked_objects.keys())

        if obj.id in all_ids:
            return True

        return False

    def is_object_blocked(self, obj) -> bool:
        """
        State if a given object is blocked,

        :param obj: designator of given object
        """
        if not self.is_object_in_use(obj):
            return False

        if self.can_robot_do_collab_action(self.blocked_objects[obj.id]):
            return False

        return True

    def can_robot_do_collab_action(self, obj_info):
        return len(obj_info["robots"]) < obj_info["robots_needed"]


