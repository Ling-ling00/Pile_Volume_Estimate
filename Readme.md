# Multi-LiDAR Volume Estimation
This project is a ROS 2 package for estimating the volume of a pile. It uses multiple LiDARs to scan the area, merges the data into one map, finds the pile using clustering, and calculates the volume.

---

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (Humble) or later
- ROS 2 Humble
- Python 3.10+

---

## Installation
### Step 1: Clone Repository

```bash
mkdir volume_estimate
mkdir volume_estimate/src
cd volume_estimate/src
git clone https://github.com/Ling-ling00/Pile_Volume_Estimate.git .
cd ..
```
clone repository inside your_ros_ws/volume_estimate/src

### Step 2: Install ROS Dependencies
```bash
sudo apt install ros-humble-tf-transformations
sudo apt install ros-humble-sensor-msgs-py
```

### Step 3: Install Python Dependencies

```bash
# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install requirements
pip install -r requirements.txt
```

### Step 4: Build and source
at your_ros_ws/volume_estimate
```bash
colcon build
source install setup.bash
```

---

## Node Description
This system has 4 main nodes that work together.

### lidar_calibrator
This node automatically aligns your LiDARs. It reads the initial position from a config file and then uses the ICP (Iterative Closest Point) algorithm to make the alignment perfect.

**Input**
| Topic Name | Type | Description | 
|:---|:---|:---| 
| lidar_1_topic | sensor_msgs/PointCloud2 | Raw input from LiDAR sensor 1 | 
| ... | ... | ... | 
| lidar_n_topic | sensor_msgs/PointCloud2 | Raw input from LiDAR sensor n |

**Output** 
| Topic Name | Type | Description | 
|:---|:---|:---| 
| /tf_static | tf2_msgs/TFMessage | Broadcasts the calculated transform between LiDARs |

### lidar_merger
This node combines data from all LiDARs into one single pointcloud.

**Input** 
| Topic Name | Type | Description | 
|:---|:---|:---| 
| lidar_1_topic | sensor_msgs/PointCloud2 | Raw input from LiDAR sensor 1 | 
| ... | ... | ... | 
| lidar_n_topic | sensor_msgs/PointCloud2 | Raw input from LiDAR sensor n | 
| /tf_static | tf2_msgs/TFMessage | Listens for the calibration transforms |

**Output** 
| Topic Name | Type | Description | 
|:---|:---|:---| 
| /merged_cloud | sensor_msgs/PointCloud2 | The combined pointcloud of all sensors |

### pointcloud_clustering_node
This is the main processing node. It filters noise and finds the pile. Steps include:
- Height Filter: Removes the floor and ceiling.
- Downsample: Makes the data lighter to process.
- Clustering: Uses DBSCAN to group points based on position and slope angle.
- Pile Check: Selects the cluster that has a specific slope angle (looks like a pile).

**Input** 
| Topic Name | Type | Description | 
|:---|:---|:---| 
| /merged_cloud | sensor_msgs/PointCloud2 | The combined pointcloud |

**Output** 
| Topic Name | Type | Description | 
|:---|:---|:---| 
| /clustered_cloud | sensor_msgs/PointCloud2 | Full cloud with added fields: cluster_id and slope_angle | 
| /pile_cloud | sensor_msgs/PointCloud2 | Filtered cloud of only the target pile |

### volume_estimator_node
This node calculates the final volume. Steps include:
- Clustering: Uses DBSCAN to merge pile tops and slopes based on distance.
- Interpolation: Fills gaps in sparse data using linear interpolation to create a continuous surface.
- Calculation: Computes total volume by summing (Height - Floor) × Cell Area for every grid cell.

**Input** 
| Topic Name | Type | Description | 
|:---|:---|:---| 
| /pile_cloud | sensor_msgs/PointCloud2 | The filtered pile pointcloud |

**Output** 
| Topic Name | Type | Description | 
|:---|:---|:---| 
| /volume_estimate | std_msgs/Float64 | Total volume of the pile in cubic meters (m3) | 
| /voxel_heights | std_msgs/Float32MultiArray | A flattened array representing the 2D height grid |
| /merge_pile_cloud | sensor_msgs/PointCloud2 | A pointcloud show points after interpolate |

---

## Parameters
You can adjust these parameters at src/params/params.yaml and build again or adjust via `ros2 param set`.

### lidar_calibrator & lidar_merger
These parameters control how the LiDARs are loaded and aligned.

| Parameter | Type | Default | Description |
|:---|:---|:---|:---|
| config_path | String | .../lidar_pose.yaml | Absolute path to the YAML file defining LiDAR topics and initial positions. |
| max_correspondence_distance | Float | 0.2 | (lidar_calibrator Only) The max distance (meters) for matching points during ICP alignment. |

#### config file
At config path there should be yaml file for initial lidar setting including lidar topic and transformation (x y z roll pitch yaw) 
Ex:
```yaml
# topic_name: x y z roll pitch yaw
- /lidar1/lidar_points_PointCloud2: "1.2 24.25 9.0 0.0 1.57 0.0"
- lidar2/lidar_points_PointCloud2: "15.0 6.0 9.0 1.57 0.0 0.0"
- lidar3/lidar_points_PointCloud2: "20.0 6.6 9.0 -1.57 0.0 0.0"
- lidar4/lidar_points_PointCloud2: "42.5 6.0 9.0 1.57 0.0 0.0"
- lidar5/lidar_points_PointCloud2: "42.5 6.6 9.0 -1.57 0.0 0.0"
- lidar6/lidar_points_PointCloud2: "22.0 18.0 10.0 -1.57 0.0 0.0"
```

### pointcloud_clustering_node
These parameters control the noise filtering and pile detection logic.

| Parameter | Type | Default | Description |
|:---|:---|:---|:---|
| floor_z_height | Float | 0.05 | Points below this Z-height (meters) are removed (ground filter). |
| ceiling_z_height | Float | 8.0 | Points above this Z-height (meters) are removed (roof filter). |
| down_sampling_size | Float | 0.1 | Voxel size for downsampling in meters. |
| cluster_dbscan_eps | Float | 0.3 | Search radius for DBSCAN clustering. |
| cluster_dbscan_min_points| Int | 20 | Minimum points required to form a core cluster in DBSCAN. |
| cluster_min_points | Int | 50 | Minimum total points required for a valid cluster output. |
| normal_weight | Float | 2.0 | Weight given to slope angle during clustering. |
| normal_estimation_radius | Float | 0.3 | Radius used to calculate the normal vector of a point. |
| min_pile_angle | Float | 5.0 | Minimum average slope angle (degrees) to classify a cluster as a "pile". |
| max_pile_angle | Float | 45.0 | Maximum average slope angle (degrees) to classify a cluster as a "pile". |

### volume_estimator_node
These parameters control the final volume calculation grid.

| Parameter | Type | Default | Description |
|:---|:---|:---|:---|
| voxel_size | Float | 0.2 | Size of the grid cells for volume calculation (0.2 = 20cm x 20cm). |
| floor_height | Float | 0.0 | The baseline Z-height to calculate volume from. |
| cluster_dbscan_eps | Float | 1.5 | Search radius for DBSCAN clustering. |
| cluster_dbscan_min_points| Int | 20 | Minimum points required to form a core cluster in DBSCAN. |
| cluster_min_points | Int | 50 | Minimum total points required for a valid cluster output. |
| max_interpolation_dist | Float | 3.0 | Max gap size to fill during interpolation. |

---

## Usage

### Manual run

Follow these steps to run the system.

**1. Prepare Configuration**
Ensure your `lidar_pose.yaml` file is ready and the `config_path` parameter in the code points to it.

**2. Calibration (Run once)**
Run this node to generate the static transform (TF) between LiDARs.
```bash
ros2 run volume_estimate lidar_calibrator.py
```
Wait until you see "✅ All LiDARs calibrated" in the terminal.

**3. Start Merging** Once calibrated, run the merger to combine the pointclouds.
```bash
ros2 run volume_estimate merge_pointcloud.py
```

**4. Start Clustering & Filtering** Run the processing node to find the pile.
```bash
ros2 run volume_estimate clustering.py
```

**5. Start Volume Estimation** Run the estimator to calculate the volume.
```bash
ros2 run volume_estimate estimate.py 
```

**6. View Results** You can check the volume output in the terminal:
```bash
ros2 topic echo /volume_estimate
```
Or view the voxel grid heights:
```bash
ros2 topic echo /voxel_heights
```

### Using launch file

The launch file runs all 4 nodes automatically. It includes a 5-second delay after the calibrator starts to ensure the transforms are ready before the other nodes run. It also loads parameters from params.yaml.

**1. Prepare Configuration**
- Ensure your lidar_pose.yaml file is ready.
- Check that the config_path in params.yaml points to your lidar_pose.yaml file.
- Adjust any other parameters in params.yaml as needed.

**2. Run Launch File**
```bash
ros2 launch volume_estimate volume_estimate.launch.py
```

**Note: This repository does not include the simulation environment or the pile data. Please ensure your LiDAR input sources are running and publishing data before starting this system.**