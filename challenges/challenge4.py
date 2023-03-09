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

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0

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
        red_mask_center = red_mask[center_y-10:center_y+10, center_x-10:center_x+10]
        if np.any(red_mask_center):
            self.state = State.RED_DETECTED

    def scan_callback(self, msg):
        print(self.state)
        if self.state == State.DRIVE_FORWARD:
            self.to_the_wall(msg)
        elif self.state == State.ROTATING:
            self.rotate(msg)
        elif self.state == State.RED_DETECTED:
            self.to_red_wall(msg)
        
    def to_the_wall(self, msg):
        if msg.ranges[0] < 0.6:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
            self.state = State.ROTATING
        else:
            self.vel(lin_vel_percent=90)

    def rotate(self, msg):
        if msg.ranges[0] > 1.5:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
            self.state = State.DRIVE_FORWARD
        else:
            self.vel(lin_vel_percent=0, ang_vel_percent=70)

    def to_red_wall(self, msg):
        if msg.ranges[0] < 0.4:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
        else:
            self.vel(lin_vel_percent=90)

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