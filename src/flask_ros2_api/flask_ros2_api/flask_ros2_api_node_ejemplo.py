import rclpy
from rclpy.node import Node
from flask import Flask, jsonify
import threading

app = Flask(__name__)

class MiNodoROS(Node):
    def __init__(self):
        super().__init__('nodo_flask_ros')
        self.publisher_ = self.create_publisher(String, 'mi_topico', 10)
        self.get_logger().info('Nodo ROS 2 en Flask iniciado')

    def publicar_mensaje(self, mensaje):
        msg = String()
        msg.data = mensaje
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicado: {mensaje}')

# Inicializar ROS 2 en un hilo separado
def iniciar_nodo_ros():
    rclpy.init()
    nodo = MiNodoROS()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

# Ruta en Flask para enviar datos a ROS 2
@app.route('/publicar/<mensaje>', methods=['GET'])
def publicar(mensaje):
    if nodo_ros:
        nodo_ros.publicar_mensaje(mensaje)
        return jsonify({'status': 'Mensaje publicado', 'mensaje': mensaje})
    return jsonify({'status': 'Error', 'mensaje': 'Nodo ROS no inicializado'})

# Iniciar el hilo de ROS 2
nodo_ros = None
def iniciar_servidor():
    global nodo_ros
    nodo_ros = MiNodoROS()
    hilo_ros = threading.Thread(target=iniciar_nodo_ros, daemon=True)
    hilo_ros.start()
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    iniciar_servidor()

