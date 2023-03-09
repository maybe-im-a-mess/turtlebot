from tb3 import *

class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
                Twist,
                'cmd_vel',
                1) 
        
        self.odom_sub = self.create_subscription(
                Odometry,
                'odom',
                self.odom_callback, 
                qos_profile_sensor_data) 
        
        self.ang_vel_percent = 0
        self.lin_vel_percent = 0

        self.state = State.TO_THE_FIRST_WALL

    def vel(self, lin_vel_percent, ang_vel_percent=0):

        MAX_LIN_VEL = 0.26
        MAX_ANG_VEL = 1.82

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

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
        self.yaw = yaw * 180 / pi

        if self.state == State.TO_THE_FIRST_WALL:
            self.to_first_wall()
        elif self.state == State.ROTATING:
            self.rotate()
        elif self.state == State.TO_THE_SECOND_WALL:
            self.to_second_wall()
        elif self.state == State.STOP:
            self.stop()

    def to_first_wall(self):
        if self.y > 0.8:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
            self.state = State.ROTATING
        else:
            self.vel(lin_vel_percent=50)

    def rotate(self):
        if 0 < self.yaw < 180:
            self.vel(lin_vel_percent=0, ang_vel_percent=20)
        else:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
            self.state = State.TO_THE_SECOND_WALL

    def to_second_wall(self):
        if self.x < 0.1:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
            self.state = State.STOP
        else:
            self.vel(lin_vel_percent=50)

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
