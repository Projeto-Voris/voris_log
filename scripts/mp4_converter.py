#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.timer = None  # O timer será inicializado após o vídeo ser carregado
        self.cap = None
        self.bridge = CvBridge()
        self.video_path = self.declare_parameter('video_path', '').get_parameter_value().string_value
        self.resize = self.declare_parameter('resize', True).get_parameter_value().bool_value

        if not self.video_path:
            self.get_logger().error("Caminho do vídeo não fornecido. Por favor, use 'ros2 run video_publisher video_publisher_node --ros-args -p video_path:=/caminho/do/seu/video.mp4'")
            return

        if not os.path.exists(self.video_path):
            self.get_logger().error(f"Arquivo de vídeo não encontrado em: {self.video_path}")
            return

        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir o vídeo: {self.video_path}")
            self.cap = None
            return

        # Calcular a taxa de publicação com base no FPS do vídeo
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if fps > 0:
            self.timer_period = 1.0 / fps  # Segundos por frame
            self.get_logger().info(f"Publicando frames a aproximadamente {fps:.2f} FPS.")
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
        else:
            self.get_logger().warn("Não foi possível determinar o FPS do vídeo. Publicando a 30 FPS por padrão.")
            self.timer_period = 1.0 / 30.0
            self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def timer_callback(self):
        if self.cap is None:
            return

        ret, frame = self.cap.read()
        if ret:
            try:
                # Converter o frame do OpenCV (BGR) para uma mensagem ROS 2 Image (RGB)
                if self.resize:
                    # self.get_logger().info('Redimensionando o frame para 800x600')
                    frame = cv2.resize(frame, (800,600), cv2.INTER_LINEAR)
                # self.get_logger().info('Publicando frame') # Descomente para ver o log de cada publicação
                ros_image_message = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(ros_image_message)
            except Exception as e:
                self.get_logger().error(f"Erro ao converter e publicar imagem: {e}")
        else:
            # self.get_logger().info('Fim do vídeo ou erro na leitura. Reiniciando o vídeo.')
            # self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0) # Reinicia o vídeo
            # Se preferir parar ao invés de reiniciar:
            self.cap.release()
            self.get_logger().info('Vídeo concluído e recursos liberados.')
            self.destroy_node() # Opcional: Destruir o nó ao fim do vídeo
            rclpy.shutdown() # Opcional: Desligar o ROS 2


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    if node.cap is not None: # Verifica se o vídeo foi carregado com sucesso antes de spin
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()