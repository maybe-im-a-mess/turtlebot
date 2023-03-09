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

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0

        self.state = State.TO_THE_FIRST_WALL

    def vel(self, lin_vel_percent, ang_vel_percent=0):

        MAX_LIN_VEL = 0.26 
        MAX_ANG_VEL = 3

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def scan_callback(self, msg):
        if self.state == State.TO_THE_FIRST_WALL:
            self.to_first_wall(msg)
        elif self.state == State.STOP:
            self.stop()
    
    def to_first_wall(self, msg):
        if msg.ranges[0] < 0.2:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
            self.state = State.STOP
        else:
            self.vel(lin_vel_percent=50)

    def stop(self):
        self.vel(lin_vel_percent=0, ang_vel_percent=0)


def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
