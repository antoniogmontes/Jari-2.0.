import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32

class Micro(Node):
    def __init__(self):
        super().__init__('micro')

        # Subscriptores
        self.send_command_microfono = self.create_subscription(String, '/send_command_micro', self.send_commands_micro, 10)
        self.send_angle_microfono = self.create_subscription(Float32, '/send_angle_micro', self.send_angles_micro, 10)    

        # Publicadores
        self.pub_command_microfono = self.create_publisher(String, '/command_micro', 10)
        self.pub_angle_microfono = self.create_publisher(Float32, '/angle_micro', 10)

    def send_commands_micro(self, msg):
        if msg.data:  # Si hay nuevo mensaje en el tópico /send_command_micro
            self.get_logger().info('Recibiendo comnado microfono: ')
            self.get_logger().info(msg.data)

    def send_angles_micro(self, msg):
        if msg.data:  # Si hay nuevo mensaje en el tópico /send_angle_micro
            self.get_logger().info('Recibiendo angulo microfono: ')
            self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    micro = Micro()
    rclpy.spin(micro)
    micro.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
