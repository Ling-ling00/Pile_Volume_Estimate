#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import yaml
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations
from sensor_msgs_py import point_cloud2 as pc2

class LidarCalibrator(Node):
    def __init__(self):
        super().__init__('lidar_calibrator')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        self.declare_parameter('config_path', '/home/ponwalai/fibo-technovation/volume_estimate/src/params/lidar_pose.yaml')
        self.declare_parameter('max_correspondence_distance', 0.2)
        
        config_path = self.get_parameter('config_path').value
        
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.topics = [list(item.keys())[0] for item in self.config]
        self.collected_clouds = {}
        self.subs = []
        self.global_merge_cloud = np.empty((0, 3))
        
        self.get_logger().info(f"Waiting for data from: {self.topics}")

        for topic in self.topics:
            sub = self.create_subscription(
                PointCloud2, topic, 
                lambda msg, t=topic: self.cloud_callback(msg, t), 10)
            self.subs.append(sub)

    def cloud_callback(self, msg, topic):
        """
        Stored first cloud data of each topics
        """
        if topic in self.collected_clouds:
            return

        points_xyz = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points_xyz.shape[0] > 0:
            self.collected_clouds[topic] = points_xyz
            self.get_logger().info(f"Received {topic}")
        
        if len(self.collected_clouds) == len(self.topics):
            # collected all destroy subscription
            for sub in self.subs:
                self.destroy_subscription(sub)
            self.run_calibration(self.collected_clouds)

    def get_matrix_from_yaml_str(self, yaml_str):
        """
        yaml str input as: x y z roll pitch yaw
        """
        parts = [float(x) for x in yaml_str.split()]
        matrix = tf_transformations.euler_matrix(parts[3], parts[4], parts[5])
        matrix[:3, 3] = parts[:3]
        return matrix

    def run_calibration(self, clouds_dict):
        final_transforms = []
        uncalibrated = list(self.topics)
        calibrated = {} # Stores {topic: global_matrix_to_base_link}
        
        # First LiDAR
        master_topic = uncalibrated.pop(0)
        master_yaml_str = next(item[master_topic] for item in self.config if master_topic in item)
        master_matrix = self.get_matrix_from_yaml_str(master_yaml_str)
        
        calibrated[master_topic] = master_matrix
        final_transforms.append(self.make_tf_msg(master_matrix, "base_link", master_topic))

        # Add First points to the Global Merge Cloud
        master_points = clouds_dict[master_topic]
        hp_master = np.hstack((master_points, np.ones((master_points.shape[0], 1))))
        self.global_merge_cloud = (master_matrix @ hp_master.T).T[:, :3]
        
        self.get_logger().info(f"First LiDAR set: {master_topic}")

        # GREEDY EXPANSION: Add remaining LiDARs one by one
        while uncalibrated:
            best_source = None
            min_dist = float('inf')
            
            # Find uncalibrated LiDAR closest to the center of the current map
            map_center = np.mean(self.global_merge_cloud, axis=0)
            
            for u_topic in uncalibrated:
                u_yaml_str = next(item[u_topic] for item in self.config if u_topic in item)
                u_guess_matrix = self.get_matrix_from_yaml_str(u_yaml_str)
                u_pos = u_guess_matrix[:3, 3]
                
                dist = np.linalg.norm(u_pos - map_center)
                if dist < min_dist:
                    min_dist = dist
                    best_source = u_topic
                    best_init_matrix = u_guess_matrix
                    
            
            self.get_logger().info(f"Aligning {best_source} to global map (Dist to center: {min_dist:.2f}m)")

            # using ICP to refine position
            # target: global map (fix)
            target_o3d = o3d.geometry.PointCloud()
            target_o3d.points = o3d.utility.Vector3dVector(self.global_merge_cloud)
            
            # source: uncalibrateed cloud
            source_o3d = o3d.geometry.PointCloud()
            source_o3d.points = o3d.utility.Vector3dVector(clouds_dict[best_source])

            max_correspondence_distance = self.get_parameter("max_correspondence_distance").value
            reg = o3d.pipelines.registration.registration_icp(
                source_o3d, target_o3d, max_correspondence_distance, best_init_matrix,
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            
            global_matrix = reg.transformation
            calibrated[best_source] = global_matrix
            uncalibrated.remove(best_source)
            
            # Add new points to the map
            source_points = clouds_dict[best_source]
            hp_source = np.hstack((source_points, np.ones((source_points.shape[0], 1))))
            transformed_source = (global_matrix @ hp_source.T).T[:, :3]
            self.global_merge_cloud = np.vstack((self.global_merge_cloud, transformed_source))
            
            final_transforms.append(self.make_tf_msg(global_matrix, "base_link", best_source))

        self.static_broadcaster.sendTransform(final_transforms)
        self.get_logger().info("✅ All LiDARs calibrated and merged into global map.")

    def make_tf_msg(self, matrix, parent, child):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(matrix[0, 3])
        t.transform.translation.y = float(matrix[1, 3])
        t.transform.translation.z = float(matrix[2, 3])
        q = tf_transformations.quaternion_from_matrix(matrix)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        return t

def main(args=None):
    rclpy.init(args=args)
    node = LidarCalibrator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()