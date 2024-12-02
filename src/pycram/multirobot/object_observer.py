from typing import Dict


class ObjectObserver:
    """
    Class to observe the state of objects that are currently in use and shouldn't be accessed by another robot
    """

    blocked_objects: Dict[int, Dict[str, str]] = {}
    """
    Variable that stores the array of objects
    """

    def __init__(self):
        """
        Currently does nothing
        """

    def block_object(self, obj, robot_name: str):
        """
        Add an object to the observer list
        """

        object_details = {"robot": robot_name,
                          "name": obj.name}

        self.blocked_objects[obj.id] = object_details

    def release_object(self, obj):
        """
        Remove an object from the observer list
        """

        self.blocked_objects.pop(obj.id)

    def is_object_blocked(self, obj) -> bool:
        """
        State if a given object is blocked,

        :param obj: designator of given object
        """
        all_ids = list(self.blocked_objects.keys())

        if obj.id in all_ids:
            return True

        return False
