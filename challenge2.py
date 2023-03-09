from tb3 import *


class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
                Twist,      # message type
                'cmd_vel',  # topic name
                1)          # history depth
        

        self.scan_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,  # function to run upon message arrival
                qos_profile_sensor_data)  # allows packet loss
        

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.state = State.TO_THE_FIRST_WALL

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 3  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent


    def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """
        if self.state == State.TO_THE_FIRST_WALL:
            self.to_first_wall(msg)
        elif self.state == State.ROTATING:
            self.rotate(msg)
        elif self.state == State.TO_THE_SECOND_WALL:
            self.to_second_wall(msg)
        elif self.state == State.STOP:
            self.stop()
    
    def to_first_wall(self, msg):
        if msg.ranges[0] < 0.3:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
            self.state = State.ROTATING
        else:
            self.vel(lin_vel_percent=50)

    def rotate(self, msg):
        if msg.ranges[-90] > 0.4:
            self.vel(lin_vel_percent=0, ang_vel_percent=85)
        else:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
            self.state = State.TO_THE_SECOND_WALL

    def to_second_wall(self, msg):
        if msg.ranges[0] < 0.2:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
        else:
            self.vel(lin_vel_percent=50)

    def stop(self):
        self.vel(lin_vel_percent=0, ang_vel_percent=0)


    def stop_wall(self, msg):
        if msg.ranges[0] < 0.2:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
        else:
            self.vel(lin_vel_percent=50)

    def turn_wall(self, msg):
        flage = 'driving'

        if msg.ranges[0] < 0.3:
            self.vel(lin_vel_percent=0, ang_vel_percent=0)
            flag = 'stop'
        if flag == 'stop' and msg.ranges[-90] > 0.4:
            self.vel(lin_vel_percent=0, ang_vel_percent=85)
            flag = 'stop'
        if flag == 'driving':
            self.vel(lin_vel_percent=50)


def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print('waiting for messages...')

    try:
        rclpy.spin(tb3)  # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
