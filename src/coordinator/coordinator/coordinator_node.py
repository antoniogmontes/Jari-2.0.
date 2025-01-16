import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32

class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        
        # Subscriptores
        # Servidor Web FLASK
        self.sub_button = self.create_subscription(String, '/button', self.go_button, 10)
        
        # Arduino
        self.sub_command_arduino = self.create_subscription(String, '/command_ard', self.commands_arduino, 10)
        	
	    # Microfono + Altavoz
        self.command_microfono = self.create_subscription(String, '/command_micro', self.commands_micro, 10)
        self.angle_microfono = self.create_subscription(Float32, '/angle_micro', self.angles_micro, 10)

	    # Camara
        self.sub_camera = self.create_subscription(String, '/camera', self.go_camera, 10)
	

        # Publicaciones
	    # Servidor Web FLASK
	
	    # Arduino
        self.pub_send_commnad_arduino = self.create_publisher(String, '/send_command_ard', 10)
        
	    # Microfono + Altavoz
        self.pub_send_command_microfono = self.create_publisher(String, '/send_command_micro', 10)
        self.pub_send_angle_microfono = self.create_publisher(Float32, '/send_angle_micro', 10)

	    # Camara
        self.pub_send_camera = self.create_publisher(String, '/camera', 10)
	
    def go_button(self, msg):
        if msg.data:  # Si el botón está presionado
            self.get_logger().info('Botón presionado, iniciando acción')    

    def commands_arduino(self, msg):
        if msg.data:  # Si hay un nuevo mensaje en el tópico /command_arduino
            self.get_logger().info('Recibiendo comando arduino, iniciando accion')
            self.get_logger().info(msg.data)
            self.pub_send_command_arduino.publish(String(data=msg.data))
            self.pub_send_command_microfono.publish(String(data=msg.data))

    def commands_micro(self,msg):
        if msg.data:  # Si hay un nuevo mensaje en el tópico /command_micro
            self.get_logger().info('Recibiendo emoción microfono, iniciando accion')
            self.get_logger().info(msg.data)
            self.pub_send_command_arduino.publish(String(data=msg.data))
            self.pub_send_command_microfono.publish(String(data=msg.data))

    def angles_micro(self, msg):
        if msg.data:  # Si hay nuevo mensaje en el tópico /angle_micro
            self.get_logger().info('Recibiendo angulo microfono, iniciando accion')
            self.get_logger().info(msg.data)
            value = msg.data    # Convertir float a string
            cadena = str(value)
            self.pub_send_command_arduino.publish(String(data=cadena))
            self.pub_send_angle_microfono.publish(String(data=msg.data))

    def go_camera(self, msg):
        if msg.data:  # Si hay un nuevo mensaje de la camara /camera
            self.get_logger().info('Recibiendo mensaje camara')


def main(args=None):
    rclpy.init(args=args)
    coordinator = Coordinator()
    rclpy.spin(coordinator)
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
