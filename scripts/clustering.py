#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import open3d as o3d
from sensor_msgs_py import point_cloud2 as pc2
from sklearn.cluster import DBSCAN 

class PointCloudClustering(Node):
    def __init__(self):
        super().__init__('pointcloud_clustering_node')

        # --- Parameters ---
        self.declare_parameter('floor_z_height', 0.05)
        self.declare_parameter('cluster_dbscan_eps', 0.3) 
        self.declare_parameter('cluster_min_points', 20)
        self.declare_parameter('normal_weight', 2.0) 
        self.declare_parameter('normal_estimation_radius', 0.3)
        self.declare_parameter('min_pile_angle', 5.0)
        self.declare_parameter('max_pile_angle', 45.0)

        # --- Publishers ---
        self.publisher_ = self.create_publisher(PointCloud2, "/clustered_cloud", 10)
        self.publisher_pile = self.create_publisher(PointCloud2, "/pile_cloud", 10)

        self.create_subscription(PointCloud2, "/merged_cloud", self.cloud_callback, 10)

        self.get_logger().info('Clustering Node started (Per-Cluster Averaging Mode).')

    def cloud_callback(self, msg: PointCloud2):
        # Read data
        full_data_np = pc2.read_points_numpy(msg, skip_nans=True)
        if full_data_np.size == 0:
            return

        points_xyz = full_data_np[:, :3]

        # filter height
        floor_z = self.get_parameter('floor_z_height').value
        mask = points_xyz[:, 2] > floor_z
        object_points = points_xyz[mask]
        original_data_filtered = full_data_np[mask]

        # return if size less than min point (no cluster)
        if object_points.shape[0] < self.get_parameter('cluster_min_points').value:
            return

        # ---------- CLUSTERING -------------
        # Estimate normal vector and angle
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(object_points)
        radius = self.get_parameter('normal_estimation_radius').value
        o3d_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
        
        normals = np.asarray(o3d_cloud.normals)
        
        # Calculate Slope Angle
        nz = np.abs(normals[:, 2])
        angles_rad = np.arccos(np.clip(nz, 0.0, 1.0))
        angles_deg = np.degrees(angles_rad)

        # Calculate Cost (Feature Space: [X, Y, Z, Angle*w])
        w = self.get_parameter('normal_weight').value
        feature_space = np.hstack((object_points, angles_rad.reshape(-1, 1) * w))

        # SKLEARN DBSCAN
        eps = self.get_parameter('cluster_dbscan_eps').value
        min_pts = self.get_parameter('cluster_min_points').value
        db = DBSCAN(eps=eps, min_samples=min_pts).fit(feature_space)
        labels = db.labels_

        # Prepare data for publish
        cluster_id_col = labels.astype(np.float32).reshape(-1, 1)
        slope_angle_col = angles_deg.astype(np.float32).reshape(-1, 1)
        final_array = np.column_stack((original_data_filtered, cluster_id_col, slope_angle_col))

        new_fields = list(msg.fields)
        last_offset = new_fields[-1].offset + 4 
        new_fields.append(PointField(name='cluster_id', offset=last_offset, datatype=PointField.FLOAT32, count=1))
        new_fields.append(PointField(name='slope_angle', offset=last_offset+4, datatype=PointField.FLOAT32, count=1))

        # Publish full clustered cloud
        out_msg = pc2.create_cloud(msg.header, new_fields, final_array.tolist())
        self.publisher_.publish(out_msg)

        # ---------- FILTERING -------------
        unique_labels = np.unique(labels)
        pile_points_list = []
        
        min_angle = self.get_parameter('min_pile_angle').value
        max_angle = self.get_parameter('max_pile_angle').value

        for label in unique_labels:
            if label == -1: continue

            cluster_mask = (labels == label)
            avg_angle = np.mean(angles_deg[cluster_mask])
            num_point = final_array[cluster_mask].shape[0]
            if min_angle < avg_angle < max_angle and num_point >= min_pts:
                pile_points_list.append(final_array[cluster_mask])
                self.get_logger().info(f"Cluster {label} passed: Avg Slope {avg_angle:.2f}°")

        # Publish pile points
        if pile_points_list:
            pile_points_combined = np.vstack(pile_points_list)
            pile_msg = pc2.create_cloud(msg.header, new_fields, pile_points_combined.tolist())
            self.publisher_pile.publish(pile_msg)
        else:
            self.get_logger().warn("No clusters met the average pile angle criteria.", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudClustering()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()