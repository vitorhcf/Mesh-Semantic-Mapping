#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Built-in ROS 2 library specifically for transforming sensor messages
import tf2_sensor_msgs.tf2_sensor_msgs 

class SAMCloudTransformer(Node):
    def __init__(self):
        super().__init__('sam_cloud_transformer')
        
        # 1. Setup TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 2. Define the target frame ('odom' or 'map')
        self.target_frame = 'map' 
        
        # 3. Setup Publisher (The transformed cloud)
        self.pc_publisher = self.create_publisher(
            PointCloud2, 
            '/sam_pointcloud_filtered', 
            10
        )
        
        # 4. Setup Subscriber (The incoming cloud from SAM)
        self.pc_subscriber = self.create_subscription(
            PointCloud2, 
            '/sam3_perception_interface/pcl', 
            self.cloud_callback, 
            10
        )
        
        self.get_logger().info(f"Node started! Waiting for SAM point clouds to transform to '{self.target_frame}'...")

    def cloud_callback(self, msg: PointCloud2):
        source_frame = msg.header.frame_id
        
        # If the cloud is already in the target frame, just republish it
        if source_frame == self.target_frame:
            self.pc_publisher.publish(msg)
            return

        try:
            # 5. Look up the exact transform from the camera frame to 'odom'/'map'
            # rclpy.time.Time() gets the latest available transform
            transform_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 6. Apply the transformation instantly using the built-in library
            transformed_cloud = tf2_sensor_msgs.do_transform_cloud(msg, transform_stamped)
            
            # 7. Publish the newly transformed cloud
            self.pc_publisher.publish(transformed_cloud)
            self.get_logger().info(f"Successfully transformed point cloud from {source_frame} to {self.target_frame}.")
            
        except Exception as ex:
            # If the TF tree isn't fully broadcasted yet, it will catch the error here
            self.get_logger().warn(
                f"Could not transform from {source_frame} to {self.target_frame}: {ex}", 
                throttle_duration_sec=2.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = SAMCloudTransformer()
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