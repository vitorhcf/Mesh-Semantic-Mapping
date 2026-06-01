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
        
        self.mesh_scale = 1.0

        self.table_file = '/ros2_ws/table_mesh.stl'
        self.chairs_file = '/ros2_ws/chair_mesh.stl'

        self.get_logger().info(f"Looking for table mesh at: {self.table_file}")
        self.get_logger().info(f"Looking for chairs mesh at: {self.chairs_file}")

        self.timer = self.create_timer(1.0, self.publish_semantic_map)
        self.has_published = False
        self.shutdown_timer = None

        self.get_logger().info("Node started, waiting for STL files.")
        
    def create_marker(self, marker_id, name_space, color_rgba, file_path):

    def create_marker(self, marker_id, name_space, color_rgba, file_path):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = Time()
        marker.ns = name_space
        marker.id = marker_id
        
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = f"file://{file_path}"
        marker.scale.x = self.mesh_scale
        marker.scale.y = self.mesh_scale
        marker.scale.z = self.mesh_scale
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
        if self.has_published:
            return

        marker_array = MarkerArray()
        id_counter = 0
        
        if os.path.exists(self.table_file):
            marker_table = self.create_marker(id_counter, "dining_table", [0.8, 0.4, 0.1, 0.9], self.table_file)
            marker_array.markers.append(marker_table)
            id_counter += 1
        else:
            self.get_logger().warn(f"Missing table mesh: {self.table_file}", throttle_duration_sec=5.0)

        if os.path.exists(self.chairs_file):
            marker_chairs = self.create_marker(id_counter, "chairs", [0.1, 0.5, 0.9, 0.9], self.chairs_file)
            marker_array.markers.append(marker_chairs)
            id_counter += 1
        else:
            self.get_logger().warn(f"Missing chairs mesh: {self.chairs_file}", throttle_duration_sec=5.0)

        if len(marker_array.markers) > 0:
            self.publisher_.publish(marker_array)
            self.has_published = True
            self.get_logger().info(f"Published {len(marker_array.markers)} semantic meshes.")
            self.shutdown_timer = self.create_timer(0.5, self.shutdown_after_publish)

    def shutdown_after_publish(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
            self.shutdown_timer = None

        self.get_logger().info("Publish complete. Shutting down to preserve the markers.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapPublisher()
    
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
