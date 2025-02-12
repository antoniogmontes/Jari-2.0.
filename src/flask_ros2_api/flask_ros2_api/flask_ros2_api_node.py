import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32

class Flask(Node):
    def __init__(self):
        super().__init__('flask')
        
        # Subscriptores
        # Servidor Web FLASK
        self.sub_flask = self.create_subscription(String, '/send_cmd', self.flask_callback, 10)

        # Publicaciones
        self.pub_flask = self.create_publisher(String, '/button', 10)

	
    def flask_callback(self, msg):
        if msg.data:  # Si el botón está presionado
            self.get_logger().info('Botón presionado, iniciando acción')
            # variable para guardar la cadena
            message_in = msg.data

    def flask_callback_to_coord(self, msg):
        if msg.data:  # Si el botón está presionado
            self.get_logger().info('Botón presionado, iniciando acción')
            # variable para guardar la cadena
            message_in = msg.data

def main(args=None):
    rclpy.init(args=args)
    flask = Flask()
    rclpy.spin(flask)
    flask.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
