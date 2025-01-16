import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32

class Arduino(Node):
    def __init__(self):
        super().__init__('arduino')

        # Subscriptores
        self.send_command_arduino = self.create_subscription(String, '/send_command_ard', self.send_commands_arduino, 10)
        
        # Publicadores
        self.pub_command_arduino = self.create_publisher(String, '/command_ard', 10)

    def send_commands_arduino(self, msg):
        if msg.data:  # Si hay nuevo mensaje en el tópico /send_command_ard
            self.get_logger().info('Recibiendo comando arduino: ')
            self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    arduino = Arduino()
    rclpy.spin(arduino)
    arduino.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
