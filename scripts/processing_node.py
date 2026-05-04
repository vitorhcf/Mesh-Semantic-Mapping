#!/usr/bin/env python3
import glob
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
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
        self.idle_timeout_sec = 1.0
        self.received_clouds = {}
        self.last_cloud_time = None
        self.finalized = False
        self.shutdown_timer = None
        self.finalize_timer = self.create_timer(0.2, self._finalize_if_idle)
        self.output_dir = '/ros2_ws/semantic_meshes'
        self.table_mesh_path = '/ros2_ws/table_mesh.stl'
        self.chair_mesh_path = '/ros2_ws/chair_mesh.stl'
        os.makedirs(self.output_dir, exist_ok=True)
        self._cleanup_previous_outputs()
        
        self.get_logger().info("Processing node started")

    def _cleanup_previous_outputs(self):
        stale_paths = [
            self.table_mesh_path,
            self.chair_mesh_path,
        ]
        stale_paths.extend(glob.glob(os.path.join(self.output_dir, '*.stl')))

        removed_files = 0
        for path in stale_paths:
            if os.path.exists(path):
                os.remove(path)
                removed_files += 1

        self.get_logger().info(f"Limpeza inicial concluída. {removed_files} ficheiro(s) STL antigos removidos.")

    def pc_callback(self, msg):
        if self.finalized:
            return

        # Extrair classe do frame_id (ex: odom_dining_table)
        frame_parts = msg.header.frame_id.split('_')
        if len(frame_parts) < 2:
            self.get_logger().error("Frame ID não contém classe")
            return
        class_name = '_'.join(frame_parts[1:])

        self.received_clouds[class_name] = msg
        self.last_cloud_time = self.get_clock().now()
        self.get_logger().info(f"Recebida nuvem para {class_name}. À espera de estabilizar antes de gerar a mesh final.")

    def _build_mesh_marker(self, msg, class_name):
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        if not points_list:
            self.get_logger().error(f"Nuvem vazia para {class_name}")
            return None

        points = np.array([[p[0], p[1], p[2]] for p in points_list])
        colors = np.array([[ ((p[3] >> 16) & 0xFF)/255.0, ((p[3] >> 8) & 0xFF)/255.0, (p[3] & 0xFF)/255.0 ] for p in points_list])
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        if pcd.is_empty():
            self.get_logger().error(f"Nuvem vazia para {class_name}")
            return None

        self.get_logger().info(f"-> Nuvem original: {len(pcd.points)} pontos.")

        pcd_limpo, _ = pcd.remove_statistical_outlier(nb_neighbors=15, std_ratio=0.5)
        pcd_limpo = pcd_limpo.voxel_down_sample(voxel_size=0.02)

        self.get_logger().info("A agrupar instâncias e a gerar Mesh (STL)...")
        labels = np.array(pcd_limpo.cluster_dbscan(eps=0.12, min_points=20, print_progress=False))

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
                    
                    mesh_cluster = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(cluster_pcd, self.alpha)
                    
                    mesh_cluster.remove_duplicated_vertices()
                    mesh_cluster.remove_duplicated_triangles()
                    
                    mesh_final += mesh_cluster

        if not mesh_final.is_empty():
            mesh_final.compute_vertex_normals()

            if len(mesh_final.triangles) > self.simplificacao_triangulos:
                self.get_logger().info(f"Simplificando de {len(mesh_final.triangles)} para {self.simplificacao_triangulos} triângulos...")
                mesh_final = mesh_final.simplify_quadric_decimation(target_number_of_triangles=self.simplificacao_triangulos)

            if class_name == 'dining_table':
                nome_saida_stl = self.table_mesh_path
            else:
                nome_saida_stl = self.chair_mesh_path
            o3d.io.write_triangle_mesh(nome_saida_stl, mesh_final)
            self.get_logger().info(f"SUCESSO: STL gravado como '{nome_saida_stl}'")
            
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
            return marker
        else:
            self.get_logger().info(f"Nenhuma mesh foi gerada para {class_name}.")
            return None

    def _finalize_if_idle(self):
        if self.finalized or self.last_cloud_time is None:
            return

        elapsed = (self.get_clock().now() - self.last_cloud_time).nanoseconds / 1e9
        if elapsed < self.idle_timeout_sec:
            return

        self.finalize_timer.cancel()

        delete_all = Marker()
        delete_all.header.frame_id = "odom"
        delete_all.action = Marker.DELETEALL
        self.marker_publisher.publish(MarkerArray(markers=[delete_all]))

        marker_array = MarkerArray()
        for class_name, msg in self.received_clouds.items():
            marker = self._build_mesh_marker(msg, class_name)
            if marker is not None:
                marker_array.markers.append(marker)

        self.finalized = True

        if marker_array.markers:
            self.marker_publisher.publish(marker_array)
            self.get_logger().info(f"Publicado MarkerArray final com {len(marker_array.markers)} mesh(es).")
        else:
            self.get_logger().info("Nenhuma mesh final foi publicada.")

        self.shutdown_timer = self.create_timer(0.5, self._shutdown_after_publish)

    def _shutdown_after_publish(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
            self.shutdown_timer = None

        self.get_logger().info("Processamento concluído. A encerrar o nó para fixar esta execução.")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
