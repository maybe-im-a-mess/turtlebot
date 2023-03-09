import math
from enum import Enum, auto
from math import pi

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from transforms3d.euler import quat2euler


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



