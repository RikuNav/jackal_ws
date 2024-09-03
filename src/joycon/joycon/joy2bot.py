import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Joy
from clearpath_platform_msgs.msg import Drive
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Joy2Bot(Node):
    def __init__(self):
        super().__init__('joy2bot')

        # Declare Parameters
        self.declare_parameter('fast.linear-speed', 1.0)
        self.declare_parameter('fast.angular-speed', 3.14)
        self.declare_parameter('slow.linear-speed', 0.4)
        self.declare_parameter('slow.angular-speed', 0.6)
        self.declare_parameter('wheels_length', 0.3765)
        self.declare_parameter('wheels_radius', 0.095)

        # Get parameters into Variables
        self.max_fast_linear_speed = self.get_parameter('fast.linear-speed').get_parameter_value().double_value
        self.max_fast_angular_speed = self.get_parameter('fast.angular-speed').get_parameter_value().double_value
        self.max_slow_linear_speed = self.get_parameter('slow.linear-speed').get_parameter_value().double_value
        self.max_slow_angular_speed = self.get_parameter('slow.angular-speed').get_parameter_value().double_value
        self.wheels_length = self.get_parameter('wheels_length').get_parameter_value().double_value
        self.wheels_radius = self.get_parameter('wheels_radius').get_parameter_value().double_value

        # Declare Variables
        self.left_bumper = 0
        self.right_bumper = 0
        self.left_stick_x = 0.
        self.left_stick_y = 0.

        # Timers
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.timer_callback)

        # Joycon Subscription
        self.create_subscription(
            Joy,
            '/joy_teleop/joy',
            self.joycon_callback,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        # Jackal Publisher
        self.publisher = self.create_publisher(Drive, 
                                                '/platform/motors/cmd_drive', 
                                                QoSProfile(
                                                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                    history=QoSHistoryPolicy.KEEP_LAST,
                                                    depth=1
                                                )
                                            )

    # Message transformation to left and right speeds
    def joycon_callback(self, msg):
        # Getting raw joystick data
        self.left_stick_x = msg.axes[0]
        self.left_stick_y = msg.axes[1]

        self.left_bumper = msg.buttons[4]
        self.right_bumper = msg.buttons[5]

    def timer_callback(self):
        # Getting Corresponding Linear and Angular Speeds
        linear_speed = 0.
        angular_speed = 0.

        if self.left_bumper:
            linear_speed = self.max_slow_linear_speed*self.left_stick_y
            angular_speed = self.max_slow_angular_speed*self.left_stick_x
        elif self.right_bumper:
            linear_speed = self.max_fast_linear_speed*self.left_stick_y
            angular_speed = self.max_fast_angular_speed*self.left_stick_x

        # Inverse Kinematics
        left_wheel_speed = (2*linear_speed-self.wheels_length*angular_speed)/(2*self.wheels_radius)
        right_wheel_speed = (self.wheels_length/self.wheels_radius)*angular_speed+left_wheel_speed

        # Drive message for jackal
        jackal_msg = Drive()
        jackal_msg.mode = Drive.MODE_VELOCITY
        jackal_msg.drivers = [left_wheel_speed, right_wheel_speed]

        self.publisher.publish(jackal_msg)

def main(args=None):
    rclpy.init(args=args)
    joy2bot = Joy2Bot()
    rclpy.spin(joy2bot)
    joy2bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()