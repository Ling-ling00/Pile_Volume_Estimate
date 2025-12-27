#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, Float64, MultiArrayDimension
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class VolumeEstimator(Node):
    def __init__(self):
        super().__init__('volume_estimator_node')

        self.declare_parameter('voxel_size', 0.2)  # 5cm x 5cm grid cells
        self.declare_parameter('floor_height', 0.0) # Baseline height

        self.height_pub = self.create_publisher(Float32MultiArray, "/voxel_heights", 10)
        self.volume_pub = self.create_publisher(Float64, "/volume_estimate", 10)

        self.create_subscription(PointCloud2, "/pile_cloud", self.cloud_callback, 10)

        self.get_logger().info('Volume Estimator Node started.')

    def cloud_callback(self, msg: PointCloud2):
        gen = pc2.read_points(msg, field_names=("x", "y", "z", "cluster_id"), skip_nans=True)

        pile_pts = np.column_stack((
            gen['x'], 
            gen['y'], 
            gen['z'], 
            gen['cluster_id']
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

        # Map point coordinates to grid indices
        ix = ((pile_pts[:, 0] - min_x) / voxel_size).astype(int)
        iy = ((pile_pts[:, 1] - min_y) / voxel_size).astype(int)

        # Keep only points within bounds
        valid_mask = (ix >= 0) & (ix < cols) & (iy >= 0) & (iy < rows)
        ix, iy, iz = ix[valid_mask], iy[valid_mask], pile_pts[valid_mask, 2]

        # Fill Height Map (Take max Z in each voxel)
        for x_idx, y_idx, z_val in zip(ix, iy, iz):
            if z_val > height_map[y_idx, x_idx]:
                height_map[y_idx, x_idx] = z_val

        # Calculate Volume
        voxel_area = voxel_size * voxel_size
        relative_heights = height_map - floor_z
        total_volume = np.sum(relative_heights[relative_heights > 0]) * voxel_area

        # Publish Results
        self.publish_heights(height_map, rows, cols)
        self.volume_pub.publish(Float64(data=float(total_volume)))

        self.get_logger().info(f"Estimated Volume: {total_volume:.4f} m^3", throttle_duration_sec=1.0)

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