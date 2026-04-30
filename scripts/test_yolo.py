import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import sys

class YoloImageTester(Node):
    def __init__(self):
        super().__init__('yolo_image_tester')
        self.bridge = CvBridge()
        self.fotografia_tirada = False
        
        self.get_logger().info("A carregar modelo YOLO...")
        self.yolo_model = YOLO('yolov8n-seg.pt')
        
        # Subscreve apenas à imagem RGB do TIAGo
        self.rgb_sub = self.create_subscription(
            Image, 
            '/head_front_camera/rgb/image_raw', 
            self.image_callback, 
            10
        )
        self.get_logger().info("À espera de uma imagem da câmara do simulador...")

    def image_callback(self, msg):
        if self.fotografia_tirada:
            return

        self.get_logger().info("📸 Imagem recebida! A processar com o YOLO...")
        
        # 1. Converter ROS Image para OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 2. Correr o YOLO pedindo ESPECIFICAMENTE apenas as classes 56 (chair) e 60 (dining table)
        # Ao passar a lista 'classes', ele ignora paredes, pessoas, etc.
        resultados = self.yolo_model(cv_img, classes=[56, 60], verbose=False)
        
        # 3. Desenhar a deteção na imagem
        # O método .plot() devolve a imagem com as caixas e máscaras coloridas aplicadas
        imagem_anotada = resultados[0].plot()
        
        # 4. Gravar a imagem no disco
        nome_ficheiro = "visao_do_robo_segmentada.jpg"
        cv2.imwrite(nome_ficheiro, imagem_anotada)
        
        self.get_logger().info(f"✅ SUCESSO! Imagem gravada como: {nome_ficheiro}")
        
        # Analisar o que foi encontrado para imprimir no terminal
        if resultados[0].masks is not None:
            classes_detetadas = resultados[0].boxes.cls.cpu().numpy()
            nomes = [self.yolo_model.names[int(c)] for c in classes_detetadas]
            self.get_logger().info(f"Objetos encontrados: {nomes}")
        else:
            self.get_logger().warning("Aviso: A imagem foi gravada, mas o YOLO não detetou nem mesas nem cadeiras nela!")

        # Bloqueia novas capturas e encerra
        self.fotografia_tirada = True
        sys.exit(0)

def main():
    rclpy.init()
    node = YoloImageTester()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
