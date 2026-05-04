#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import os
from builtin_interfaces.msg import Time

class SemanticMapPublisher(Node):
    def __init__(self):
        super().__init__('semantic_map_publisher')
        
        self.publisher_ = self.create_publisher(MarkerArray, '/semantic_map_meshes', 10)
        
        # O tamanho tem de ser 1.0 para manter o tamanho real gerado pelo Open3D
        self.mesh_scale = 1.0

        # Atualizado para procurar ficheiros .stl
        self.table_file = '/ros2_ws/table_mesh.stl'
        self.chairs_file = '/ros2_ws/chair_mesh.stl'
        
        # Imprime imediatamente onde vai procurar
        self.get_logger().info(f"Vou procurar a mesa em: {self.table_file}")
        self.get_logger().info(f"Vou procurar as cadeiras em: {self.chairs_file}")
                
        self.timer = self.create_timer(1.0, self.publish_semantic_map)
        
        self.get_logger().info("Nó iniciado. À procura de ficheiros STL...")

    def create_marker(self, marker_id, name_space, color_rgba, file_path):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = Time()
        marker.ns = name_space
        marker.id = marker_id
        
        # --- A GRANDE MUDANÇA ---
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        
        # O RViz exige que o caminho tenha 'file://' no início para ficheiros locais
        marker.mesh_resource = f"file://{file_path}"
        
        # A escala passa a ser 1.0, 1.0, 1.0 (o tamanho real da mesh)
        marker.scale.x = self.mesh_scale
        marker.scale.y = self.mesh_scale
        marker.scale.z = self.mesh_scale
        
        # Posição central na origem (a rotação/posição já foi guardada no STL pelo Open3D)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Cor
        marker.color.r = float(color_rgba[0])
        marker.color.g = float(color_rgba[1])
        marker.color.b = float(color_rgba[2])
        marker.color.a = float(color_rgba[3]) 
            
        return marker

    def publish_semantic_map(self):
        marker_array = MarkerArray()
        id_counter = 0
        
        # --- 1. PROCESSAR A MESA ---
        if os.path.exists(self.table_file):
            # Cor: Castanho alaranjado para a mesa
            marker_table = self.create_marker(id_counter, "dining_table", [0.8, 0.4, 0.1, 0.9], self.table_file)
            marker_array.markers.append(marker_table)
            id_counter += 1
        else:
            self.get_logger().warn(f"MESA EM FALTA! Não encontro: {self.table_file}", throttle_duration_sec=5.0)
        
        # --- 2. PROCESSAR AS CADEIRAS ---
        if os.path.exists(self.chairs_file):
            # Cor: Azul claro para as cadeiras
            marker_chairs = self.create_marker(id_counter, "chairs", [0.1, 0.5, 0.9, 0.9], self.chairs_file)
            marker_array.markers.append(marker_chairs)
            id_counter += 1
        else:
            self.get_logger().warn(f"CADEIRAS EM FALTA! Não encontro: {self.chairs_file}", throttle_duration_sec=5.0)

        # Publicar apenas se houver ficheiros processados
        if len(marker_array.markers) > 0:
            self.publisher_.publish(marker_array)
            self.get_logger().info(f"Sucesso! {len(marker_array.markers)} meshes publicadas no RViz2.", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()