import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        
        # Subscriptores
        self.sub_camera = self.create_subscription(String, '/camera', self.go_camera, 10)

        # Publicadores
        self.pub_camera = self.create_publisher(String, '/camera', 10)

    def go_camera(self, msg):
        if msg.data:  # Si hay un mesaje en el topico /camera
            self.get_logger().info('Nuevo mensaje de la camara recibido')
            self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()