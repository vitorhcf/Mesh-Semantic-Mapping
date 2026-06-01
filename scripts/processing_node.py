#!/usr/bin/env python3
import glob
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
import open3d as o3d
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_sensor_msgs.tf2_sensor_msgs

class ProcessingNode(Node):
    def __init__(self):
        super().__init__('processing_node')

        # Setup TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'map'

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        
        self.subscription = self.create_subscription(PointCloud2, '/sam3_perception_interface/pcl', self.pc_callback, 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/semantic_map_meshes', marker_qos)
        self.bbox_publisher = self.create_publisher(MarkerArray, '/semantic_map_bboxes', marker_qos)

        self.table_aliases = {
            'dining_table',
            'tables',
            'table',
            'mesa',
            'mesas',
        }
        
        # Runtime settings
        self.alpha = 0.12  # Open3D alpha shape value: lower = more detail, higher = more solid
        self.simplification_target_triangles = 1500
        self.idle_timeout_sec = 1.0
        self.mesh_voxel_size = 0.02
        self.bbox_voxel_size = 0.01
        self.bbox_cluster_eps = 0.10
        self.bbox_cluster_min_points = 30
        self.table_plane_distance_threshold = 0.015
        self.received_clouds = {}
        self.last_cloud_time = None
        self.finalized = False
        self.finalize_timer = self.create_timer(0.2, self._finalize_if_idle)
        self.republish_timer = self.create_timer(1.0, self._republish_markers)
        self.output_dir = '/ros2_ws/semantic_meshes'
        self.table_mesh_path = '/ros2_ws/table_mesh.stl'
        self.chair_mesh_path = '/ros2_ws/chair_mesh.stl'
        self.latest_mesh_markers = MarkerArray()
        self.latest_bbox_markers = MarkerArray()
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

        self.get_logger().info(f"Initial cleanup complete. Removed {removed_files} stale STL files.")

    def pc_callback(self, msg):
        if self.finalized:
            return

        source_frame = msg.header.frame_id
        
        # Try to transform the point cloud to the map frame
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            # Apply transformation
            msg_transformed = tf2_sensor_msgs.do_transform_cloud(msg, transform_stamped)
            msg = msg_transformed
            self.get_logger().info(f"Transformed point cloud from {source_frame} to {self.target_frame}")
        except Exception as ex:
            self.get_logger().warn(
                f"Could not transform from {source_frame} to {self.target_frame}: {ex}. Using original frame.",
                throttle_duration_sec=5.0
            )
        
        # Default class name (can be enhanced with semantic info)
        class_name = 'dining_table'

        self.received_clouds[class_name] = msg
        self.last_cloud_time = self.get_clock().now()
        self.get_logger().info(f"Received cloud for {class_name}. Waiting to stabilize before final mesh generation.")

    def _point_cloud_from_msg(self, msg, class_name):
        field_names = [field.name for field in msg.fields]
        if not all(name in field_names for name in ("x", "y", "z")):
            self.get_logger().error(f"PointCloud2 for {class_name} does not contain x,y,z fields: {field_names}")
            return None

        use_rgb = False
        use_rgb_separate = False
        if "rgb" in field_names:
            point_fields = ("x", "y", "z", "rgb")
            use_rgb = True
        elif {"r", "g", "b"}.issubset(field_names):
            point_fields = ("x", "y", "z", "r", "g", "b")
            use_rgb_separate = True
        else:
            point_fields = ("x", "y", "z")

        try:
            points_list = list(pc2.read_points(msg, field_names=point_fields, skip_nans=True))
        except Exception as ex:
            self.get_logger().error(
                f"Falha ao ler PointCloud2 de {class_name}: {ex}. Campos disponíveis: {field_names}"
            )
            return None

        if not points_list:
            self.get_logger().error(f"Empty point cloud for {class_name}")
            return None

        points = np.array([[p[0], p[1], p[2]] for p in points_list], dtype=np.float64)

        if use_rgb:
            colors = np.array([
                [((int(p[3]) >> 16) & 0xFF) / 255.0,
                 ((int(p[3]) >> 8) & 0xFF) / 255.0,
                 (int(p[3]) & 0xFF) / 255.0]
                for p in points_list
            ], dtype=np.float64)
        elif use_rgb_separate:
            colors = np.array([
                [p[3] / 255.0, p[4] / 255.0, p[5] / 255.0]
                for p in points_list
            ], dtype=np.float64)
        else:
            colors = np.ones((points.shape[0], 3), dtype=np.float64) * 0.75

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd

    def _preprocess_point_cloud(self, pcd, voxel_size):
        pcd_limpo, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
        return pcd_limpo.voxel_down_sample(voxel_size=voxel_size)

    def _cluster_point_cloud(self, pcd, eps, min_points):
        if pcd.is_empty():
            return []

        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        if len(labels) == 0 or labels.max() == -1:
            return []

        clusters = []
        for label in range(labels.max() + 1):
            idx_cluster = np.where(labels == label)[0]
            if len(idx_cluster) == 0:
                continue
            clusters.append(pcd.select_by_index(idx_cluster))
        return clusters

    def _select_primary_cluster(self, pcd, class_name):
        clusters = self._cluster_point_cloud(
            pcd,
            eps=self.bbox_cluster_eps,
            min_points=self.bbox_cluster_min_points,
        )
        if not clusters:
            return pcd

        scored_clusters = []
        for cluster in clusters:
            aabb = cluster.get_axis_aligned_bounding_box()
            extents = aabb.get_extent()
            volume = float(extents[0] * extents[1] * extents[2])
            height = float(extents[2])
            if (0.10 < height < 2.0) and (0.01 < volume < 5.0) and (len(cluster.points) > 30):
                score = len(cluster.points)
                if class_name == 'dining_table':
                    score += volume * 100.0
                scored_clusters.append((score, cluster))

        if not scored_clusters:
            return max(clusters, key=lambda cluster: len(cluster.points))
        return max(scored_clusters, key=lambda item: item[0])[1]

    def _rotation_matrix_to_quaternion(self, rotation_matrix):
        m = rotation_matrix
        trace = float(np.trace(m))
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (m[2, 1] - m[1, 2]) * s
            y = (m[0, 2] - m[2, 0]) * s
            z = (m[1, 0] - m[0, 1]) * s
        else:
            if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
                w = (m[2, 1] - m[1, 2]) / s
                x = 0.25 * s
                y = (m[0, 1] + m[1, 0]) / s
                z = (m[0, 2] + m[2, 0]) / s
            elif m[1, 1] > m[2, 2]:
                s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
                w = (m[0, 2] - m[2, 0]) / s
                x = (m[0, 1] + m[1, 0]) / s
                y = 0.25 * s
                z = (m[1, 2] + m[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
                w = (m[1, 0] - m[0, 1]) / s
                x = (m[0, 2] + m[2, 0]) / s
                y = (m[1, 2] + m[2, 1]) / s
                z = 0.25 * s

        quat = np.array([x, y, z, w], dtype=np.float64)
        quat /= np.linalg.norm(quat)
        return Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))

    def _estimate_tabletop_bbox(self, cluster_pcd):
        plane_model, inliers = cluster_pcd.segment_plane(
            distance_threshold=self.table_plane_distance_threshold,
            ransac_n=3,
            num_iterations=1000,
        )
        if len(inliers) < 50:
            return None

        plane_points = np.asarray(cluster_pcd.select_by_index(inliers).points)
        all_points = np.asarray(cluster_pcd.points)
        if len(plane_points) == 0 or len(all_points) == 0:
            return None

        normal = np.array(plane_model[:3], dtype=np.float64)
        normal /= np.linalg.norm(normal)
        if normal[2] < 0.0:
            normal *= -1.0

        plane_center = plane_points.mean(axis=0)
        projected_plane_points = plane_points - np.outer((plane_points - plane_center) @ normal, normal)
        centered_plane_points = projected_plane_points - plane_center
        covariance = centered_plane_points.T @ centered_plane_points
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        order = np.argsort(eigenvalues)[::-1]
        axis_x = eigenvectors[:, order[0]]
        axis_y = eigenvectors[:, order[1]]
        axis_x /= np.linalg.norm(axis_x)
        axis_y /= np.linalg.norm(axis_y)
        axis_z = normal

        if np.dot(np.cross(axis_x, axis_y), axis_z) < 0.0:
            axis_y *= -1.0

        rotation = np.column_stack((axis_x, axis_y, axis_z))
        plane_local = (plane_points - plane_center) @ rotation
        all_local = (all_points - plane_center) @ rotation

        min_xy = plane_local[:, :2].min(axis=0)
        max_xy = plane_local[:, :2].max(axis=0)
        min_z = all_local[:, 2].min()
        max_z = all_local[:, 2].max()

        extent = np.array([
            max_xy[0] - min_xy[0],
            max_xy[1] - min_xy[1],
            max_z - min_z,
        ], dtype=np.float64)
        local_center = np.array([
            (min_xy[0] + max_xy[0]) / 2.0,
            (min_xy[1] + max_xy[1]) / 2.0,
            (min_z + max_z) / 2.0,
        ], dtype=np.float64)
        world_center = plane_center + rotation @ local_center

        footprint_thickness = max(0.03, self.table_plane_distance_threshold * 2.0)
        footprint_local_center = np.array([
            (min_xy[0] + max_xy[0]) / 2.0,
            (min_xy[1] + max_xy[1]) / 2.0,
            plane_local[:, 2].mean(),
        ], dtype=np.float64)
        footprint_center = plane_center + rotation @ footprint_local_center

        return {
            'table_obb': {
                'center': world_center,
                'extent': extent,
                'rotation': rotation,
            },
            'tabletop_bbox': {
                'center': footprint_center,
                'extent': np.array([
                    max_xy[0] - min_xy[0],
                    max_xy[1] - min_xy[1],
                    footprint_thickness,
                ], dtype=np.float64),
                'rotation': rotation,
            },
        }

    def _estimate_bbox_data(self, pcd, class_name):
        pcd_bbox = self._preprocess_point_cloud(pcd, voxel_size=self.bbox_voxel_size)
        if pcd_bbox.is_empty():
            return []

        cluster_pcd = self._select_primary_cluster(pcd_bbox, class_name)
        if cluster_pcd.is_empty():
            return []

        if class_name == 'dining_table':
            tabletop_data = self._estimate_tabletop_bbox(cluster_pcd)
            if tabletop_data is not None:
                return [
                    ('tabletop_bbox', tabletop_data['tabletop_bbox'], (1.0, 0.55, 0.0)),
                    ('table_obb', tabletop_data['table_obb'], (0.0, 0.8, 0.2)),
                ]

        obb = cluster_pcd.get_oriented_bounding_box()
        return [
            ('object_obb', {
                'center': np.asarray(obb.center),
                'extent': np.asarray(obb.extent),
                'rotation': np.asarray(obb.R),
            }, (0.0, 0.8, 0.2))
        ]

    def _build_bbox_markers(self, msg, class_name, marker_id_start):
        pcd = self._point_cloud_from_msg(msg, class_name)
        if pcd is None or pcd.is_empty():
            return []

        bbox_specs = self._estimate_bbox_data(pcd, class_name)
        if not bbox_specs:
            self.get_logger().info(f"Could not estimate bounding boxes for {class_name}.")
            return []

        markers = []
        for offset, (namespace_suffix, bbox_data, color) in enumerate(bbox_specs):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = msg.header.stamp
            marker.ns = f"{class_name}_{namespace_suffix}"
            marker.id = marker_id_start + offset
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(bbox_data['center'][0])
            marker.pose.position.y = float(bbox_data['center'][1])
            marker.pose.position.z = float(bbox_data['center'][2])
            marker.pose.orientation = self._rotation_matrix_to_quaternion(bbox_data['rotation'])
            marker.scale.x = float(max(bbox_data['extent'][0], 0.01))
            marker.scale.y = float(max(bbox_data['extent'][1], 0.01))
            marker.scale.z = float(max(bbox_data['extent'][2], 0.01))
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = 0.35
            markers.append(marker)

            extent = bbox_data['extent']
            self.get_logger().info(
                f"{class_name}/{namespace_suffix}: "
                f"{extent[0]:.2f}m x {extent[1]:.2f}m x {extent[2]:.2f}m"
            )

        return markers

    def _build_mesh_marker(self, msg, class_name):
        pcd = self._point_cloud_from_msg(msg, class_name)
        if pcd is None or pcd.is_empty():
            return None

        self.get_logger().info(f"-> Original point cloud: {len(pcd.points)} points.")

        pcd_limpo = self._preprocess_point_cloud(pcd, voxel_size=self.mesh_voxel_size)

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
                
                # Heuristic filter for furniture-sized clusters
                if (0.10 < altura < 2.0) and (0.01 < volume < 5.0) and (len(idx_cluster) > 30):
                    self.get_logger().info(f"Processing cluster {i}: height={height:.2f}m, volume={volume:.2f}m³")
                    
                    mesh_cluster = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(cluster_pcd, self.alpha)
                    
                    mesh_cluster.remove_duplicated_vertices()
                    mesh_cluster.remove_duplicated_triangles()
                    
                    mesh_final += mesh_cluster

        if not mesh_final.is_empty():
            mesh_final.compute_vertex_normals()

            if len(mesh_final.triangles) > self.simplification_target_triangles:
                self.get_logger().info(f"Simplifying from {len(mesh_final.triangles)} to {self.simplification_target_triangles} triangles...")
                mesh_final = mesh_final.simplify_quadric_decimation(target_number_of_triangles=self.simplification_target_triangles)

            if class_name == 'dining_table':
                output_stl_name = self.table_mesh_path
            else:
                output_stl_name = self.chair_mesh_path
            o3d.io.write_triangle_mesh(output_stl_name, mesh_final)
            self.get_logger().info(f"SUCCESS: STL written to '{output_stl_name}'")
            
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = msg.header.stamp
            marker.ns = class_name
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = f"file://{output_stl_name}"
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
            self.get_logger().info(f"No mesh was generated for {class_name}.")
            return None

    def _finalize_if_idle(self):
        if self.finalized or self.last_cloud_time is None:
            return

        elapsed = (self.get_clock().now() - self.last_cloud_time).nanoseconds / 1e9
        if elapsed < self.idle_timeout_sec:
            return

        self.finalize_timer.cancel()

        delete_all = Marker()
        delete_all.header.frame_id = "map"
        delete_all.action = Marker.DELETEALL
        self.marker_publisher.publish(MarkerArray(markers=[delete_all]))
        self.bbox_publisher.publish(MarkerArray(markers=[delete_all]))

        marker_array = MarkerArray()
        bbox_marker_array = MarkerArray()
        for class_name, msg in self.received_clouds.items():
            try:
                bbox_markers = self._build_bbox_markers(msg, class_name, len(bbox_marker_array.markers))
                bbox_marker_array.markers.extend(bbox_markers)

                marker = self._build_mesh_marker(msg, class_name)
                if marker is not None:
                    marker_array.markers.append(marker)
            except Exception as ex:
                self.get_logger().error(
                    f"Error processing point cloud for {class_name}: {ex}"
                )
                continue

        self.finalized = True

        self.latest_mesh_markers = marker_array
        self.latest_bbox_markers = bbox_marker_array

        if self.latest_mesh_markers.markers:
            self.marker_publisher.publish(self.latest_mesh_markers)
            self.get_logger().info(f"Published final MarkerArray with {len(marker_array.markers)} mesh(es).")
        else:
            self.get_logger().info("No final mesh was published.")

        if self.latest_bbox_markers.markers:
            self.bbox_publisher.publish(self.latest_bbox_markers)
            self.get_logger().info(f"Published final MarkerArray with {len(bbox_marker_array.markers)} bounding box(es).")
        else:
            self.get_logger().info("No final bounding box was published.")

        self.get_logger().info("Processing complete. Node remains active for RViz visualization.")

    def _republish_markers(self):
        if not self.finalized:
            return

        if self.latest_mesh_markers.markers:
            self.marker_publisher.publish(self.latest_mesh_markers)

        if self.latest_bbox_markers.markers:
            self.bbox_publisher.publish(self.latest_bbox_markers)

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
