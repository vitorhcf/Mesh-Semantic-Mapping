#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointField
import message_filters
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
from ultralytics import YOLO
import sys
import cv2

# [NOVO] Importações para ler a árvore de Transforms (TF2)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Adicionar para publicar PointCloud2
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class TiagoVisionSnapshot(Node):
    def __init__(self):
        super().__init__('tiago_vision_snapshot')
        self.bridge = CvBridge()
        
        self.get_logger().info("A carregar modelo YOLO...")
        self.yolo_model = YOLO('/ros2_ws/yolov8n-seg.pt')
        
        # [NOVO] Configurar o escuta do TF2 para saber onde o robô está
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Adicionar publisher para PointCloud2
        self.pc_publisher = self.create_publisher(PointCloud2, '/object_pointclouds', 10)
        
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/head_front_camera/rgb/camera_info', self.camera_info_cb, 10)

        self.rgb_sub = message_filters.Subscriber(self, Image, '/head_front_camera/rgb/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/head_front_camera/depth/image_raw')
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.process_vision_cb)

        self.completed = False
        self.shutdown_timer = None
        
        self.get_logger().info("Nó iniciado! À procura de mesas (60) e cadeiras (56)...")

    def camera_info_cb(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]

    def process_vision_cb(self, rgb_msg, depth_msg):
        if self.completed:
            return

        if self.fx is None:
            self.get_logger().info("À espera das intrínsecas da câmara...")
            return

        # [NOVO] Pedir a transformação entre o mundo ('odom') e a câmara
        frame_da_camara = depth_msg.header.frame_id
        try:
            # Pede a transformação exata (odom <- camara)
            trans = self.tf_buffer.lookup_transform(
                'odom', 
                frame_da_camara, 
                rclpy.time.Time()
            )
        except Exception as ex:
            # Nos primeiros segundos o TF pode ainda não estar pronto
            self.get_logger().warn(f"A aguardar localização do robô (TF não está pronto)...", throttle_duration_sec=2.0)
            return

        self.get_logger().info("Imagens e Localização (TF) recebidas e sincronizadas!")

        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        cv_depth = np.nan_to_num(cv_depth, nan=0.0, posinf=0.0, neginf=0.0)

        depth_noise_std_m = 0.04  
        if np.issubdtype(cv_depth.dtype, np.integer):
            noise = np.random.normal(0.0, depth_noise_std_m * 1000.0, cv_depth.shape).astype(np.float32)
            cv_depth = cv_depth.astype(np.float32) + noise
        else:
            noise = np.random.normal(0.0, depth_noise_std_m, cv_depth.shape).astype(np.float32)
            cv_depth = cv_depth.astype(np.float32) + noise
        cv_depth = np.clip(cv_depth, 0.0, None)

        results = self.yolo_model(cv_rgb, verbose=False)
        
        imagem_anotada = results[0].plot()
        caminho_imagem = "/ros2_ws/yolo_output_detections.jpg"
        cv2.imwrite(caminho_imagem, imagem_anotada)

        if results[0].masks is None:
            self.get_logger().info("YOLO: Olhei, mas não encontrei objetos.")
            return

        classes_detetadas = results[0].boxes.cls.cpu().numpy()
        nomes_vistos = [self.yolo_model.names[int(cls)] for cls in classes_detetadas]
        
        mascaras_por_classe = {}
        for i, cls in enumerate(classes_detetadas):
            cls_int = int(cls)
            if cls_int == 60 or cls_int == 56:  
                nome_base = self.yolo_model.names[cls_int].replace(' ', '_')
                mascara_atual = results[0].masks[i].data[0].cpu().numpy() > 0.5
                
                if nome_base not in mascaras_por_classe:
                    mascaras_por_classe[nome_base] = mascara_atual
                else:
                    mascaras_por_classe[nome_base] = np.logical_or(mascaras_por_classe[nome_base], mascara_atual)

        objetos_extraidos = 0
        
        # [NOVO] Construir a Matriz de Transformação 4x4 do TF2
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w
        
        # Converter Quaternion do TF2 numa matriz de rotação 3x3 usando NumPy
        matriz_rotacao = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])
        
        # Montar a matriz final 4x4
        matriz_tf_4x4 = np.eye(4)
        matriz_tf_4x4[:3, :3] = matriz_rotacao
        matriz_tf_4x4[0, 3] = tx
        matriz_tf_4x4[1, 3] = ty
        matriz_tf_4x4[2, 3] = tz

        for nome_classe, mascara_agrupada in mascaras_por_classe.items():
            self.get_logger().info(f"A extrair e transformar: {nome_classe}...")

            depth_mascarado = cv_depth * mascara_agrupada
            v, u = np.where(depth_mascarado > 0)
            
            if len(v) == 0:
                continue

            z = depth_mascarado[v, u]
            if z.dtype != np.float32: z = z / 1000.0 

            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy
            
            colors = cv_rgb[v, u] / 255.0 
            colors = colors[:, ::-1] 

            pontos_3d = np.vstack((x, y, z)).T

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pontos_3d)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            # [NOVO] O momento da magia: Isto converte todos os pontos da Câmara para Odom instantaneamente
            pcd.transform(matriz_tf_4x4)

            # Publicar a nuvem de pontos como PointCloud2
            header = depth_msg.header
            header.frame_id = f'odom_{nome_classe}'  # Já transformado para odom e com classe
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)
            colors_uint = (colors * 255).astype(np.uint8)
            rgb_uint = (colors_uint[:, 0].astype(np.uint32) << 16) | \
                      (colors_uint[:, 1].astype(np.uint32) << 8) | \
                      colors_uint[:, 2].astype(np.uint32)

            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]

            cloud_points = [
                (float(x), float(y), float(z), np.uint32(rgb))
                for (x, y, z), rgb in zip(points, rgb_uint)
            ]
            cloud = pc2.create_cloud(header, fields, cloud_points)
            self.pc_publisher.publish(cloud)
            
            self.get_logger().info(f"Publicado nuvem de pontos para {nome_classe} ({len(pcd.points)} pontos)")
            objetos_extraidos += 1

        if objetos_extraidos > 0:
            self.get_logger().info(f"Publicado {objetos_extraidos} nuvens de pontos.")
            self.completed = True
            self.shutdown_timer = self.create_timer(0.5, self.shutdown_after_snapshot)

    def shutdown_after_snapshot(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
            self.shutdown_timer = None

        self.get_logger().info("Snapshot concluído. A encerrar o nó para manter esta captura fixa.")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = TiagoVisionSnapshot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
