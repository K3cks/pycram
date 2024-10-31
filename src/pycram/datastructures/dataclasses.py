from __future__ import annotations

from dataclasses import dataclass

from matplotlib import pyplot as plt
from std_msgs.msg import ColorRGBA
from typing_extensions import List, Optional, Tuple, Callable, Dict, Any, Union, TYPE_CHECKING
from .enums import JointType, Shape
from .pose import Pose, Point
from abc import ABC, abstractmethod

if TYPE_CHECKING:
    from ..description import Link
    from ..world_concepts.world_object import Object
    from ..world_concepts.constraints import Attachment


def get_point_as_list(point: Point) -> List[float]:
    """
    Returns the point as a list.

    :param point: The point.
    :return: The point as a list
    """
    return [point.x, point.y, point.z]


@dataclass
class Color:
    """
    Dataclass for storing rgba_color as an RGBA value.
    The values are stored as floats between 0 and 1.
    The default rgba_color is white. 'A' stands for the opacity.
    """
    R: float = 1
    G: float = 1
    B: float = 1
    A: float = 1

    @classmethod
    def from_list(cls, color: List[float]):
        """
        Sets the rgba_color from a list of RGBA values.

        :param color: The list of RGBA values
        """
        if len(color) == 3:
            return cls.from_rgb(color)
        elif len(color) == 4:
            return cls.from_rgba(color)
        else:
            raise ValueError("Color list must have 3 or 4 elements")

    @classmethod
    def from_rgb(cls, rgb: List[float]):
        """
        Sets the rgba_color from a list of RGB values.

        :param rgb: The list of RGB values
        """
        return cls(rgb[0], rgb[1], rgb[2], 1)

    @classmethod
    def from_rgba(cls, rgba: List[float]):
        """
        Sets the rgba_color from a list of RGBA values.

        :param rgba: The list of RGBA values
        """
        return cls(rgba[0], rgba[1], rgba[2], rgba[3])

    def get_rgba(self) -> List[float]:
        """
        Returns the rgba_color as a list of RGBA values.

        :return: The rgba_color as a list of RGBA values
        """
        return [self.R, self.G, self.B, self.A]

    def get_rgb(self) -> List[float]:
        """
        Returns the rgba_color as a list of RGB values.

        :return: The rgba_color as a list of RGB values
        """
        return [self.R, self.G, self.B]

    @staticmethod
    def get_color_from_string(color):
        """
        Retrieve a color based on string

        :param color: Name of a color
        """

        converted_color = ColorRGBA()
        value = lambda x: x / 255

        if color == 'pink':
            return ColorRGBA(r=value(255), g=value(0), b=value(255), a=1.0)
        elif color == 'black':
            return ColorRGBA(r=value(0), g=value(0), b=value(0), a=1.0)
        elif color == 'white':
            return ColorRGBA(r=value(255), g=value(255), b=value(255), a=1.0)
        elif color == 'red':
            return ColorRGBA(r=value(255), g=value(0), b=value(0), a=1.0)
        elif color == 'green':
            return ColorRGBA(r=value(0), g=value(255), b=value(0), a=1.0)
        elif color == 'blue':
            return ColorRGBA(r=value(0), g=value(0), b=value(255), a=1.0)
        elif color == 'yellow':
            return ColorRGBA(r=value(255), g=value(255), b=value(0), a=1.0)
        elif color == 'cyan':
            return ColorRGBA(r=value(0), g=value(255), b=value(255), a=1.0)

        return converted_color

    @staticmethod
    def gaussian_color_map(value, min_val=0.0, max_val=0.4, colormap='viridis'):
        """
        Maps a Gaussian-scaled value to a `ColorRGBA` object for color visualization.

        This function normalizes `value` within a specified range and maps it to an RGB color
        using a specified colormap. The alpha (opacity) is set to full (1.0).

        Args:
            value (float): The Gaussian value to map, expected within `min_val` and `max_val`.
            min_val (float, optional): Minimum value of the mapping range. Defaults to 0.0.
            max_val (float, optional): Maximum value of the mapping range. Defaults to 0.4.
            colormap (str, optional): The name of the Matplotlib colormap to use. Defaults to 'viridis'.

        Returns:
            ColorRGBA: A `ColorRGBA` object where the RGB channels correspond to the color
                       mapped from `value` using the specified colormap, and alpha is 1.0.
        """
        clamped_value = max(min(value, max_val), min_val)
        normalized_value = (clamped_value - min_val) / (max_val - min_val)

        color_map = plt.get_cmap(colormap)
        rgba_color = color_map(normalized_value)

        return ColorRGBA(r=rgba_color[0], g=rgba_color[1], b=rgba_color[2], a=1.0)


@dataclass
class AxisAlignedBoundingBox:
    """
    Dataclass for storing an axis-aligned bounding box.
    """
    min_x: float
    min_y: float
    min_z: float
    max_x: float
    max_y: float
    max_z: float

    @classmethod
    def from_min_max(cls, min_point: List[float], max_point: List[float]):
        """
        Sets the axis-aligned bounding box from a minimum and maximum point.

        :param min_point: The minimum point
        :param max_point: The maximum point
        """
        return cls(min_point[0], min_point[1], min_point[2], max_point[0], max_point[1], max_point[2])

    def get_min_max_points(self) -> Tuple[Point, Point]:
        """
        Returns the axis-aligned bounding box as a tuple of minimum and maximum points.

        :return: The axis-aligned bounding box as a tuple of minimum and maximum points
        """
        return self.get_min_point(), self.get_max_point()

    def get_min_point(self) -> Point:
        """
        Returns the axis-aligned bounding box as a minimum point.

        :return: The axis-aligned bounding box as a minimum point
        """
        return Point(self.min_x, self.min_y, self.min_z)

    def get_max_point(self) -> Point:
        """
        Returns the axis-aligned bounding box as a maximum point.

        :return: The axis-aligned bounding box as a maximum point
        """
        return Point(self.max_x, self.max_y, self.max_z)

    def get_min_max(self) -> Tuple[List[float], List[float]]:
        """
        Returns the axis-aligned bounding box as a tuple of minimum and maximum points.

        :return: The axis-aligned bounding box as a tuple of minimum and maximum points
        """
        return self.get_min(), self.get_max()

    def get_min(self) -> List[float]:
        """
        Returns the minimum point of the axis-aligned bounding box.

        :return: The minimum point of the axis-aligned bounding box
        """
        return [self.min_x, self.min_y, self.min_z]

    def get_max(self) -> List[float]:
        """
        Returns the maximum point of the axis-aligned bounding box.

        :return: The maximum point of the axis-aligned bounding box
        """
        return [self.max_x, self.max_y, self.max_z]


@dataclass
class CollisionCallbacks:
    on_collision_cb: Callable
    no_collision_cb: Optional[Callable] = None


@dataclass
class MultiBody:
    base_visual_shape_index: int
    base_pose: Pose
    link_visual_shape_indices: List[int]
    link_poses: List[Pose]
    link_masses: List[float]
    link_inertial_frame_poses: List[Pose]
    link_parent_indices: List[int]
    link_joint_types: List[JointType]
    link_joint_axis: List[Point]
    link_collision_shape_indices: List[int]


@dataclass
class VisualShape(ABC):
    rgba_color: Color
    visual_frame_position: List[float]

    @abstractmethod
    def shape_data(self) -> Dict[str, Any]:
        """
        Returns the shape data of the visual shape (e.g. half extents for a box, radius for a sphere).
        """
        pass

    @property
    @abstractmethod
    def visual_geometry_type(self) -> Shape:
        """
        Returns the visual geometry type of the visual shape (e.g. box, sphere).
        """
        pass


@dataclass
class BoxVisualShape(VisualShape):
    half_extents: List[float]

    def shape_data(self) -> Dict[str, List[float]]:
        return {"halfExtents": self.half_extents}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.BOX

    @property
    def size(self) -> List[float]:
        return self.half_extents


@dataclass
class SphereVisualShape(VisualShape):
    radius: float

    def shape_data(self) -> Dict[str, float]:
        return {"radius": self.radius}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.SPHERE


@dataclass
class CapsuleVisualShape(VisualShape):
    radius: float
    length: float

    def shape_data(self) -> Dict[str, float]:
        return {"radius": self.radius, "length": self.length}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.CAPSULE


@dataclass
class CylinderVisualShape(CapsuleVisualShape):

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.CYLINDER


@dataclass
class MeshVisualShape(VisualShape):
    scale: List[float]
    file_name: str

    def shape_data(self) -> Dict[str, Union[List[float], str]]:
        return {"meshScale": self.scale, "meshFileName": self.file_name}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.MESH


@dataclass
class PlaneVisualShape(VisualShape):
    normal: List[float]

    def shape_data(self) -> Dict[str, List[float]]:
        return {"normal": self.normal}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.PLANE


@dataclass
class State(ABC):
    pass


@dataclass
class LinkState(State):
    constraint_ids: Dict[Link, int]


@dataclass
class JointState(State):
    position: float


@dataclass
class ObjectState(State):
    pose: Pose
    attachments: Dict[Object, Attachment]
    link_states: Dict[int, LinkState]
    joint_states: Dict[int, JointState]


@dataclass
class WorldState(State):
    simulator_state_id: int
    object_states: Dict[str, ObjectState]
