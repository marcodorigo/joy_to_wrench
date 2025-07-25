import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped, Wrench, Vector3

class JoyToWrenchNode(Node):
    def __init__(self):
        super().__init__('joy_to_wrench')

        # Declare and get the wrench topic param
        self.declare_parameter('ho_wrench_topic', '/falcon_joystick/joystick_wrench')
        wrench_topic = self.get_parameter('ho_wrench_topic').get_parameter_value().string_value

        # Publisher for wrench
        self.wrench_pub = self.create_publisher(WrenchStamped, wrench_topic, 10)

        # Subscriber to joystick
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info(f'Publishing wrench messages to: {wrench_topic}')

        # Force magnitude (N)
        self.force_mag = 5.0

    def joy_callback(self, msg: Joy):
        force_x = 0.0
        force_y = 0.0
        force_z = 0.0

        # Buttons array: check length to avoid errors
        buttons = msg.buttons

        # X axis (forward/backwards)
        if len(buttons) > 8:
            if buttons[5]:  # Up button -> +X force
                force_x += self.force_mag
            if buttons[8]:  # Down button -> -X force
                force_x -= self.force_mag

        # Y axis (left/right)
        if len(buttons) > 7:
            if buttons[6]:  # Left button -> -Y force
                force_y -= self.force_mag
            if buttons[7]:  # Right button -> +Y force
                force_y += self.force_mag

        # Z axis (up/down)
        if len(buttons) > 1:
            if buttons[0]:  # A button -> +Z force
                force_z += self.force_mag
            if buttons[1]:  # B button -> -Z force
                force_z -= self.force_mag

        # Construct wrench message
        wrench = Wrench()
        wrench.force = Vector3(x=force_x, y=force_y, z=force_z)
        wrench.torque = Vector3(x=0.0, y=0.0, z=0.0)  # No torque applied here

        wrench_stamped = WrenchStamped()
        wrench_stamped.header.stamp = self.get_clock().now().to_msg()
        wrench_stamped.header.frame_id = 'base_link'  # or whatever your robot's base frame is
        wrench_stamped.wrench = wrench

        # Publish wrench
        self.wrench_pub.publish(wrench_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToWrenchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
