import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32, Int32

class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        
        # Arduino
        self.sub_command_arduino = self.create_subscription(String, '/command_ard', self.commands_arduino, 10)
        self.pub_send_commnad_arduino = self.create_publisher(Int32, '/send_command_ard', 10)

        # Servidor Web FLASK
        self.sub_button = self.create_subscription(String, '/button', self.go_button, 10)
        	
	    # Microfono + Altavoz
        self.command_microfono = self.create_subscription(String, '/command_micro', self.commands_micro, 10)
        self.angle_microfono = self.create_subscription(Float32, '/angle_micro', self.angles_micro, 10)

        self.pub_send_command_microfono = self.create_publisher(String, '/send_command_micro', 10)
        self.pub_send_angle_microfono = self.create_publisher(Float32, '/send_angle_micro', 10)

	    # Camera
        self.sub_camera = self.create_subscription(String, '/camera_sub', self.go_camera, 10)
        self.pub_send_camera = self.create_publisher(String, '/camera_pub', 10)
	
    def decoder_msg(self, mensaje):
        """Decodifica el mensaje recibido y devuelve un número."""
        diccionario_comandos = {
            "serious": 1,
            "very_happy": 2,
            "happy": 3,
            "very_sad": 4,
            "sad": 5,
            "disgusted": 6,
            "angry": 7,
            "scared": 8,
            "surprised": 9,
            "forward": 10,
            "backward": 11,
            "left": 12,
            "right": 13,
            "turn_right": 15,
            "turn_left": 16,
            "stop": 14,
            "off": 0
        }
        return diccionario_comandos.get(mensaje.lower(), -1)  # Retorna -1 si no reconoce el mensaje  

    def commands_arduino(self, msg):
        if msg.data:  # Si hay un nuevo mensaje en el tópico /command_arduino
            self.get_logger().info('Recibiendo comando arduino, iniciando accion')
            
    # Callback de la llegada de un mensaje atraves del microfono
    def commands_micro(self,msg):
        if msg.data:  # Si hay un nuevo mensaje en el tópico /command_micro
            self.get_logger().info('Recibiendo emoción microfono, iniciando accion')
            self.get_logger().info(msg.data)

            """Callback que recibe un string, lo decodifica y publica el valor numérico."""
            number = self.decodificar_mensaje(msg.data)
            if number != -1:
                msgNum = Int32()
                msgNum.data = number
                self.pub_send_command_arduino.publish(msgNum)
                self.pub_send_command_microfono.publish(String(data=msg.data))
                self.get_logger().info(f"Mensaje recibido: '{msg.data}', enviado como: {number}")
            else:
                self.get_logger().warn(f"Mensaje no reconocido: '{msg.data}'")


    def angles_micro(self, msg):
        if msg.data:  # Si hay nuevo mensaje en el tópico /angle_micro
            self.get_logger().info('Recibiendo angulo microfono, iniciando accion')
            self.get_logger().info(msg.data)

            if msg.data < 0:
                self.pub_send_command_arduino.publish(15)
            elif msg.data > 0:
                self.pub_send_command_arduino.publish(16)
            else:
                self.pub_send_command_arduino.publish(0)

    def go_camera(self, msg):
        if msg.data:  # Si hay un nuevo mensaje de la camara /camera
            self.get_logger().info('Recibiendo mensaje camara')

    def go_button(self, msg):
        if msg.data:  # Si el botón está presionado
            self.get_logger().info('Botón presionado, iniciando acción')  

def main(args=None):
    rclpy.init(args=args)
    coordinator = Coordinator()
    rclpy.spin(coordinator)
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()