#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Float64, MultiArrayDimension
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

# Processing libraries
from sklearn.cluster import DBSCAN
from scipy.interpolate import griddata
from scipy.spatial import cKDTree

class VolumeEstimator(Node):
    def __init__(self):
        super().__init__('volume_estimator_node')

        self.declare_parameter('voxel_size', 0.2)  # 5cm x 5cm grid cells
        self.declare_parameter('floor_height', 0.0) # Baseline height
        # Clustering to merge top of pile and pile slope
        self.declare_parameter('cluster_dbscan_eps', 1.50)
        self.declare_parameter('cluster_dbscan_min_points', 20)
        self.declare_parameter('cluster_min_points', 50)
        self.declare_parameter('max_interpolation_dist', 3.0)

        self.height_pub = self.create_publisher(Float32MultiArray, "/voxel_heights", 10)
        self.volume_pub = self.create_publisher(Float64, "/volume_estimate", 10)
        self.cluster_pub = self.create_publisher(PointCloud2, "/merge_pile_cloud", 10)

        self.create_subscription(PointCloud2, "/pile_cloud", self.cloud_callback, 10)

        self.get_logger().info('Volume Estimator Node started.')

    def cloud_callback(self, msg: PointCloud2):
        gen = pc2.read_points(msg, field_names=("x", "y", "z", "cluster_id"), skip_nans=True)

        pile_pts = np.column_stack((
            gen['x'], 
            gen['y'], 
            gen['z']
        ))

        if pile_pts.size == 0:
            self.volume_pub.publish(Float64(data=0.0))
            return

        # Discretize into Voxel Grid
        voxel_size = self.get_parameter('voxel_size').value
        min_x = min(pile_pts[:, 0])
        max_x = max(pile_pts[:, 0])

        min_y = min(pile_pts[:, 1])
        max_y = max(pile_pts[:, 1])

        floor_z = self.get_parameter('floor_height').value

        # Calculate grid dimensions
        cols = int((max_x - min_x) / voxel_size)
        rows = int((max_y - min_y) / voxel_size)

        # Initialize height map with floor height
        height_map = np.full((rows, cols), floor_z, dtype=np.float32)

        # Clustering using distance only (to merge top of pile and pile slope)
        eps = self.get_parameter('cluster_dbscan_eps').value
        min_samples = self.get_parameter('cluster_dbscan_min_points').value

        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(pile_pts)
        labels = clustering.labels_
        unique_labels = set(labels)
        if -1 in unique_labels: 
            unique_labels.remove(-1)

        # Interpolate and Volume
        total_volume = 0.0
        
        min_points = self.get_parameter('cluster_min_points').value
        max_interp_dist = self.get_parameter('max_interpolation_dist').value
        cell_area = voxel_size * voxel_size
        
        # Create coordinate matrices
        global_grid_x, global_grid_y = np.mgrid[
            min_x : (min_x + cols*voxel_size) : voxel_size,
            min_y : (min_y + rows*voxel_size) : voxel_size
        ]

        global_grid_x = global_grid_x[:cols, :rows]
        global_grid_y = global_grid_y[:cols, :rows]

        point_list = []
        
        for i, label in enumerate(unique_labels):
            cluster_pts = pile_pts[labels == label]
            if len(cluster_pts) < min_points: 
                continue

            tree = cKDTree(cluster_pts[:, :2])
            
            # Interpolate (Linear fills the Convex Hull)
            try:
                grid_z = griddata(
                    points=cluster_pts[:, :2],
                    values=cluster_pts[:, 2],
                    xi=(global_grid_x, global_grid_y),
                    method='linear',
                    fill_value=floor_z         
                )
            except Exception:
                continue

            # Flatten
            flat_x = global_grid_x.flatten()
            flat_y = global_grid_y.flatten()
            flat_z = grid_z.flatten()

            mask_valid_z = ~np.isnan(flat_z) & (flat_z >= floor_z)

            indices_to_check = np.where(mask_valid_z)[0]
            if len(indices_to_check) == 0:
                continue

            points_to_check = np.column_stack((flat_x[indices_to_check], flat_y[indices_to_check]))
            dists, _ = tree.query(points_to_check)

            valid_indices = indices_to_check[dists < max_interp_dist]
            cluster_vol = 0.0
            
            for idx in valid_indices:
                z_val = flat_z[idx]
                height_diff = z_val - floor_z
                
                # Update Volume
                cluster_vol += height_diff * cell_area

                # Update Global Height Map
                r = idx // cols
                c = idx % cols
                
                # Store relative height in the map (or absolute Z if preferred, here using relative)
                if height_diff > height_map[r, c]:
                    height_map[r, c] = height_diff
                
                point_list.append([flat_x[idx], flat_y[idx], flat_z[idx], i])

            total_volume += cluster_vol

        self.publish_heights(height_map, rows, cols)
        self.volume_pub.publish(Float64(data=float(total_volume)))
        self.publish_clustered_cloud(np.array(point_list), msg.header)
        
        self.get_logger().info(f"Estimated Volume: {total_volume:.4f} m^3", throttle_duration_sec=1.0)

    def publish_clustered_cloud(self, points_with_ids, header):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = pc2.create_cloud(header, fields, points_with_ids)
        self.cluster_pub.publish(cloud_msg)

    def publish_heights(self, height_map, rows, cols):
        msg = Float32MultiArray()
        
        # Define MultiArray layout
        msg.layout.dim.append(MultiArrayDimension(label="rows", size=rows, stride=rows*cols))
        msg.layout.dim.append(MultiArrayDimension(label="cols", size=cols, stride=cols))
        
        msg.data = height_map.flatten().tolist()
        self.height_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VolumeEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()