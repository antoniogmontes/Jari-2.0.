import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32, Int32
import subprocess
import signal
import os
import psutil  # Agrega esta librería (instalar con `pip install psutil` si no la tienes)

class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        
        """Creacion de subscriptores y publicadores"""
        # Arduino
        self.sub_cmd_arduino = self.create_subscription(String, '/cmd_ard', self.commands_arduino, 10)
        self.pub_send_cmd_arduino = self.create_publisher(Int32, '/send_cmd_ard', 10)

        # Servidor Web FLASK
        self.sub_cmd_flask = self.create_subscription(String, '/cmd_flask', self.commands_flask, 10)
        self.sub_mode = self.create_subscription(String, '/mode', self.change_mode, 10)
        self.pub_send_cmd_flask = self.create_publisher(String, '/send_cmd_flask', 10)

	    # Microfono + Altavoz
        self.sub_cmd_micro = self.create_subscription(String, '/cmd_micro', self.commands_micro, 10)
        self.sub_angle_micro = self.create_subscription(Float32, '/angle_micro', self.angles_micro, 10)

        self.pub_send_cmd_micro = self.create_publisher(String, '/send_cmd_micro', 10)
        self.pub_send_angle_micro = self.create_publisher(Float32, '/send_angle_micro', 10)

	    # Camera
        self.sub_camera = self.create_subscription(String, '/camera_sub', self.commands_camera, 10)
        self.pub_send_camera = self.create_publisher(String, '/camera_pub', 10)

        self.current_mode = None
        self.active_processes = {}

        # Diccionario con nodos que lanzar según el modo
        self.modes = {
            'web_server': [
                ['ros2', 'launch', 'web_server', 'web_server.launch.py']
            ],
            'micro': [
                ['ros2', 'launch', 'micro', 'micro.launch.py'],
            ],
            'cmd_msgs': [
                ['ros2', 'run', 'pkg_camera', 'node_camera']
            ],
            'conversacion_presencial': [
                ['ros2', 'launch', 'coordinador', 'normal_mode.launch.py']
            ],
            'conversacion_audios': [
                ['ros2', 'launch', 'coordinador', 'node_micro.launch.py'],
                ['ros2', 'run', 'pkg_audio', 'node_audio']
            ],
            'conversacion_msgs': [
                ['ros2', 'run', 'pkg_camera', 'node_camera']
            ],
            'arduino': [
                ['ros2', 'launch', 'coordinador', 'normal_mode.launch.py']
            ],
            'camara': [
                ['ros2', 'launch', 'coordinador', 'node_micro.launch.py'],
                ['ros2', 'run', 'pkg_audio', 'node_audio']
            ],
        }


    """Decodifica el mensaje recibido y devuelve un número."""
    def decoder_msg(self, mensaje):
        commands_list = {
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
        return commands_list.get(mensaje.lower(), -1)  # Retorna -1 si no reconoce el mensaje  

    # Callback new command from arduino
    def commands_arduino(self, msg):
        if msg.data:  # Si hay un nuevo mensaje en el tópico /command_arduino
            self.get_logger().info('Recibiendo comando arduino, iniciando accion')
            self.get_logger().info(msg.data)
            
    # Callback new command from micro
    def commands_micro(self,msg):
        if msg.data:  # Si hay un nuevo mensaje en el tópico /command_micro
            self.get_logger().info('Recibiendo emoción microfono, iniciando accion')
            self.get_logger().info(msg.data)

            """Callback que recibe un string, lo decodifica y publica el valor numérico."""
            number = self.decoder_msg(msg.data)
            if number != -1:
                msgNum = Int32()
                msgNum.data = number
                self.pub_send_cmd_arduino.publish(msgNum)
                # self.pub_send_cmd_micro.publish(String(data=msg.data))
                # self.pub_send_cmd_flask.publish(String(data=msg.data))
                self.get_logger().info(f"Mensaje recibido: '{msg.data}', enviado como: {number}")
            else:
                self.get_logger().warn(f"Mensaje no reconocido: '{msg.data}'")


    def angles_micro(self, msg):
        if msg.data:  # Si hay nuevo mensaje en el tópico /angle_micro
            self.get_logger().info('Recibiendo angulo microfono, iniciando accion')
            self.get_logger().info(msg.data)

            if msg.data < 0:
                self.pub_send_cmd_arduino.publish(15)
            elif msg.data > 0:
                self.pub_send_cmd_arduino.publish(16)
            else:
                self.pub_send_cmd_arduino.publish(0)

    # Callback new command from camera
    def commands_camera(self, msg):
        if msg.data:  # Si hay un nuevo mensaje de la camara /camera
            self.get_logger().info('Recibiendo mensaje camara')
            self.get_logger().info(msg.data)

    # Callback new command from web_server
    def commands_flask(self, msg):
        if msg.data:  # Si hay un nuevo mensaje de flask /cmd_flask
            self.get_logger().info('Recibiendo mensaje flask')
            self.get_logger().info(msg.data)
            
            """Callback que recibe un string, lo decodifica y publica el valor numérico."""
            number = self.decodificar_mensaje(msg.data)
            if number != -1:
                msgNum = Int32()
                msgNum.data = number
                self.pub_send_cmd_arduino.publish(msgNum)
                self.pub_send_cmd_micro.publish(String(data=msg.data))
                # self.pub_send_cmd_flask.publish(String(data=msg.data))
                self.get_logger().info(f"Mensaje recibido: '{msg.data}', enviado como: {number}")
            else:
                self.get_logger().warn(f"Mensaje no reconocido: '{msg.data}'")

    def change_mode(self, msg):
        if msg.data:  # Si hay un nuevo mensaje de flask /mode
            self.get_logger().info('Recibiendo mensaje flask')
            self.get_logger().info(msg.data)
        
            new_mode = msg.data.lower()

            if new_mode == self.current_mode:
                self.get_logger().info(f"El modo {new_mode} ya está activo.")
                return
        

            self.get_logger().info(f'Cambiando de modo: {self.current_mode} -> {new_mode}')
     
            self.shutdown_active_processes()
   
            if new_mode in self.modes:
                self.launch_mode(new_mode)
                self.current_mode = new_mode
            else:
                self.get_logger().warn(f"Modo desconocido: {new_mode}")


    def launch_mode(self, mode):
        if mode in self.modes:
            for cmd in self.modes[mode]:
                self.get_logger().info(f"Lanzando: {' '.join(cmd)}")
                proc = subprocess.Popen(cmd)
                self.active_processes[proc.pid] = proc
        else:
            self.get_logger().warn(f"Modo desconocido: {mode}")


    def shutdown_active_processes(self):
        if not self.active_processes:
            self.get_logger().info("No hay procesos activos para cerrar.")
            return

        self.get_logger().info("Cerrando procesos activos...")

        for pid, proc in list(self.active_processes.items()):
            self.get_logger().info(f"Intentando cerrar proceso con PID {pid}")

            # Enviar SIGINT primero (Ctrl+C simulado)
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=3)  # Espera 3 segundos
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"Proceso {pid} no respondió a SIGINT, intentando SIGTERM.")
                proc.kill()  # Intentar SIGTERM
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f"Proceso {pid} sigue activo, forzando cierre con SIGKILL.")
                    proc.kill()  # Si no responde, usar SIGKILL

            # Verificar si el proceso sigue vivo
            if psutil.pid_exists(pid):
                self.get_logger().error(f"¡El proceso {pid} sigue activo! Puede haber un problema.")

            # Finalmente, eliminar del diccionario
            self.active_processes.pop(pid, None)

        self.get_logger().info("Todos los procesos han sido cerrados correctamente.")




def main(args=None):
    rclpy.init(args=args)
    coordinator = Coordinator()
    rclpy.spin(coordinator)
    coordinator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()