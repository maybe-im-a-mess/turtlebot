import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from enum import Enum, auto
from transforms3d.euler import quat2euler
import math
import numpy as np


class State(Enum):
    TO_THE_FIRST_WALL = auto()
    ROTATING = auto()
    TO_THE_SECOND_WALL = auto()
    STOP = auto()
    DRIVE_FORWARD = auto()
    RED_DETECTED = auto()
    FIND_WAY = auto()
    FORWARD = auto()
    BACK = auto()
    LEFT = auto()
    RIGHT = auto()
    ANALYZE = auto()
    JUNCTION = auto()



