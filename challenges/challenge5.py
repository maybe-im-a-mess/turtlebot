from tb3 import *


class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            1)  

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback, 
            qos_profile_sensor_data) 

        self.camera_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_callback,
            qos_profile_sensor_data)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback, 
            qos_profile_sensor_data) 

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0

        self.last_x = []
        self.last_y = []

        self.state = State.DRIVE_FORWARD

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        MAX_LIN_VEL = 0.26 
        MAX_ANG_VEL = 1.82

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def camera_callback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red, upper_red)
        red_mask = mask1 + mask2

        center_x = int(cv_image.shape[1] / 2)
        center_y = int(cv_image.shape[0] / 2)
        red_mask_center = red_mask[center_y - 10:center_y + 10, center_x - 10:center_x + 10]
        if np.any(red_mask_center):
            self.state = State.RED_DETECTED


    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.orientation.z

        quaternion = (
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        )

        pitch, roll, yaw = quat2euler(quaternion)
        self.pitch = pitch * 180 / pi
        self.roll = roll * 180 / pi
        self.yaw = yaw * 180 / pi

    def scan_callback(self, msg):
        print(self.state)
        if self.state == State.DRIVE_FORWARD:
            self.drive_forward(msg)
        elif self.state == State.ANALYZE:
            self.analyze(msg)
        elif self.state == State.ROTATING:
            print(round(abs(self.yaw)))
            if 2 >= round(abs(self.yaw)) >= 0 or 92 >= round(abs(self.yaw)) >= 88 or 180 >= round(abs(self.yaw)) >= 177:
                self.rotate(0)
                self.state = State.DRIVE_FORWARD
        elif self.state == State.JUNCTION:
            self.junction()
        elif self.state == State.RED_DETECTED:
            self.stop()
            self.to_red_wall(msg)
        elif self.state == State.STOP:
            self.stop()

    def junction(self):
        self.last_x.append(self.x)
        self.last_y.append(self.y)
        print(self.x - self.last_x[0], self.y - self.last_y[0])
        if abs(self.x - self.last_x[0]) >= 0.0 or abs(self.y - self.last_y[0]) >= 0.0:
            self.rotate(20)
            self.last_x = []
            self.last_y = []
            self.state = State.ROTATING

    def rotate(self, angle_speed):
        self.vel(0, angle_speed)
        self.state = State.DRIVE_FORWARD

    def drive_forward(self, msg):
        if msg.ranges[0] < 0.7 and self.state != State.RED_DETECTED:
            self.state = State.ANALYZE
        elif msg.ranges[0] > 1.2 and msg.ranges[90] > 1.2 and msg.ranges[180] > 1.2 and self.state != State.RED_DETECTED:
            self.state = State.JUNCTION
        else:
            self.vel(150, 0)

    def analyze(self, msg):
        if msg.ranges[-90] > 1.1 and msg.ranges[90] > 1.1:
            self.rotate(20)
            self.state = State.ROTATING
        elif msg.ranges[-90] > msg.ranges[90]:
            self.rotate(-20)
            self.state = State.ROTATING
        else:
            self.rotate(20)
            self.state = State.ROTATING

    def to_red_wall(self, msg):
        if msg.ranges[0] > 0.5:
            self.drive_forward(msg)
        else:
            self.vel(lin_vel_percent=5, ang_vel_percent=0)
            if msg.ranges[0] <= 0.1:
                self.state = State.STOP

    def stop(self):
        self.vel(lin_vel_percent=0, ang_vel_percent=0)


if __name__ == '__main__':
    rclpy.init(args=None)

    tb3 = Tb3()
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()

