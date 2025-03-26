import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from flask import Flask, jsonify
import threading

app = Flask(__name__)

class MyWebServerNode(Node):
    def __init__(self):
        super().__init__('MyWebServerNode')
        self.publisher = self.create_publisher(String, '/topico_ejemplo', 10)
    
    def info(self, mensaje):
        msg = String()
        msg.data = mensaje
        self.publisher.publish(msg)
        return {"dato": msg}      

rclpy.init(args=None)
nodo_webServer_ros = MyWebServerNode()

# Ruta en Flask para enviar datos a ROS 2
@app.route('/publicar/<mensaje>', methods=['GET'])
def publicar(mensaje):
    if nodo_webServer_ros:
        datos = nodo_webServer_ros.info(mensaje)
        return mensaje # jsonify(datos)


def run_flask():
    app.run(host='0.0.0.0', port=5000)

def main(args=None):
    hilo_webServer = threading.Thread(target=run_flask)
    hilo_webServer.start()
    rclpy.spin(nodo_webServer_ros)
    nodo_webServer_ros.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
