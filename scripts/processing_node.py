#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time
import open3d as o3d
import numpy as np

class ProcessingNode(Node):
    def __init__(self):
        super().__init__('processing_node')
        
        self.subscription = self.create_subscription(PointCloud2, '/object_pointclouds', self.pc_callback, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/semantic_map_meshes', 10)
        
        # Configurações
        self.alpha = 0.06  # Parâmetro da Mesh: menor = mais detalhe/buracos, maior = mais sólido/bolha
        self.simplificacao_triangulos = 1500 # Quantos polígonos queremos no final
        
        self.get_logger().info("Processing node started")

    def pc_callback(self, msg):
        # Extrair classe do frame_id (ex: odom_dining_table)
        frame_parts = msg.header.frame_id.split('_')
        if len(frame_parts) < 2:
            self.get_logger().error("Frame ID não contém classe")
            return
        class_name = '_'.join(frame_parts[1:])
        
        # Converter PointCloud2 para Open3D
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        points = np.array([[p[0], p[1], p[2]] for p in points_list])
        colors = np.array([[ ((p[3] >> 16) & 0xFF)/255.0, ((p[3] >> 8) & 0xFF)/255.0, (p[3] & 0xFF)/255.0 ] for p in points_list])
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        if pcd.is_empty():
            self.get_logger().error(f"Nuvem vazia para {class_name}")
            return

        # =========================================================================
        # FASE 1 & 2: FILTRAGEM (Mantida igual à tua)
        # =========================================================================
        self.get_logger().info(f"-> Nuvem original: {len(pcd.points)} pontos.")

        # Limpeza de ruído
        pcd_limpo, _ = pcd.remove_statistical_outlier(nb_neighbors=15, std_ratio=0.5)
        pcd_limpo = pcd_limpo.voxel_down_sample(voxel_size=0.02)

        # =========================================================================
        # SEPARAÇÃO (DBSCAN) E GERAÇÃO DE MESH
        # =========================================================================
        self.get_logger().info("A agrupar instâncias e a gerar Mesh (STL)...")
        labels = np.array(pcd_limpo.cluster_dbscan(eps=0.12, min_points=20, print_progress=False))

        # Criamos uma mesh vazia para juntar todos os pedaços (mesa + cadeiras se houver)
        mesh_final = o3d.geometry.TriangleMesh()

        if len(labels) > 0 and labels.max() != -1:
            max_label = labels.max()
            
            for i in range(max_label + 1):
                idx_cluster = np.where(labels == i)[0]
                cluster_pcd = pcd_limpo.select_by_index(idx_cluster)
                
                aabb = cluster_pcd.get_axis_aligned_bounding_box()
                extents = aabb.get_extent() 
                volume = extents[0] * extents[1] * extents[2]
                altura = extents[2] 
                
                # Filtro heurístico (ajustado para deixar passar mesas e cadeiras)
                if (0.10 < altura < 2.0) and (0.01 < volume < 5.0) and (len(idx_cluster) > 30):
                    self.get_logger().info(f"Processando Cluster {i}: Altura={altura:.2f}m, Volume={volume:.2f}m³")
                    
                    # --- GERAÇÃO DA MESH ---
                    mesh_cluster = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(cluster_pcd, self.alpha)
                    
                    # Limpeza da Mesh: remover triângulos duplicados ou não conectados
                    mesh_cluster.remove_duplicated_vertices()
                    mesh_cluster.remove_duplicated_triangles()
                    
                    # Adicionar esta parte à mesh final
                    mesh_final += mesh_cluster

        # =========================================================================
        # PÓS-PROCESSAMENTO E PUBLICAÇÃO
        # =========================================================================
        if not mesh_final.is_empty():
            # 1. Calcular Normais (importante para sombras no RViz/Gazebo)
            mesh_final.compute_vertex_normals()

            # 2. Simplificação (Decimação) - Torna o ficheiro leve
            if len(mesh_final.triangles) > self.simplificacao_triangulos:
                self.get_logger().info(f"Simplificando de {len(mesh_final.triangles)} para {self.simplificacao_triangulos} triângulos...")
                mesh_final = mesh_final.simplify_quadric_decimation(target_number_of_triangles=self.simplificacao_triangulos)

            # 4. GRAVAR O FICHEIRO STL
            nome_saida_stl = f"/ros2_ws/{class_name}_mesh.stl"
            o3d.io.write_triangle_mesh(nome_saida_stl, mesh_final)
            self.get_logger().info(f"SUCESSO: STL gravado como '{nome_saida_stl}'")
            
            # Criar e publicar MarkerArray
            marker_array = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = msg.header.stamp
            marker.ns = class_name
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = f"file://{nome_saida_stl}"
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
            
            self.marker_publisher.publish(marker_array)
            self.get_logger().info(f"Publicado MarkerArray para {class_name}")
        else:
            self.get_logger().info("Nenhuma mesh foi gerada.")

def main():
    rclpy.init()
    node = ProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
