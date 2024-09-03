import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from clearpath_platform_msgs.msg import Drive
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Joy2Bot(Node):
    def __init__(self):
        super().__init__('joy2bot')

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

        #'/platform/motors/cmd_drive'
        self.publisher = self.create_publisher(Drive, 
                                                '/test', 
                                                QoSProfile(
                                                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                                    history=QoSHistoryPolicy.KEEP_LAST,
                                                    depth=1
                                                )
                                            )

    def joycon_callback(self, msg):
        left_stick_x = msg.axes[0]
        left_stick_y = msg.axes[1]

        jackal_msg = Drive()
        jackal_msg.left = left_stick_x
        jackal_msg.right = left_stick_y

        self.publisher.publish(jackal_msg)

def main(args=None):
    rclpy.init(args=args)
    joy2bot = Joy2Bot()
    rclpy.spin(joy2bot)
    joy2bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()