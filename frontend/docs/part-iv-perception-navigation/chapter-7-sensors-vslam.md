---
sidebar_position: 7
title: 'Chapter 7: The Eyes (Sensors & VSLAM)'
description: 'Understanding vision sensors and Visual Simultaneous Localization and Mapping for humanoid robots'
---

# Chapter 7: The Eyes (Sensors & VSLAM)

## Introduction

Humanoid robots depend heavily on their visual perception systems to navigate and interact with the world. This chapter explores the integration of various vision sensors and Visual Simultaneous Localization and Mapping (VSLAM) techniques that enable humanoid robots to perceive their environment, localize themselves within it, and build maps for navigation. We'll cover both the hardware aspects of vision sensors and the software algorithms that process visual information.

## Vision Sensors for Humanoid Robots

### RGB-D Cameras

RGB-D cameras provide both color information and depth data, essential for humanoid robots:

- **Intel RealSense D435i**: Popular choice with integrated IMU
- **Azure Kinect**: High-quality depth sensing with IMU
- **StereoLabs ZED**: Stereo vision with GPU-accelerated depth

#### RealSense D435i Configuration

```python
import pyrealsense2 as rs
import numpy as np

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start pipeline
pipeline.start(config)

# Get frames
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
```

### Stereo Vision Systems

Stereo vision uses two cameras to calculate depth through triangulation:

```yaml
# Stereo camera configuration
stereo_camera:
  left_camera:
    intrinsics: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    resolution: [640, 480]
    distortion: [k1, k2, p1, p2, k3]
  right_camera:
    intrinsics: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    resolution: [640, 480]
    distortion: [k1, k2, p1, p2, k3]
  baseline: 0.12  # Distance between cameras in meters
```

### Event-Based Cameras

Event-based cameras provide high temporal resolution for fast motion:

```python
class EventCamera:
    def __init__(self, device_path):
        self.device = dvs_camera.open(device_path)
        self.events = []

    def process_events(self, timestamp):
        """Process asynchronous events from event camera"""
        new_events = self.device.get_events_since(timestamp)

        for event in new_events:
            if event.polarity:
                # Positive change
                self.handle_positive_change(event.x, event.y)
            else:
                # Negative change
                self.handle_negative_change(event.x, event.y)

        return new_events
```

## Camera Calibration

### Intrinsic Calibration

Calibrating camera parameters for accurate measurements:

```python
import cv2
import numpy as np

def calibrate_camera(images, pattern_size=(9, 6)):
    """Calibrate camera using checkerboard pattern"""
    obj_points = []  # 3D points in real world space
    img_points = []  # 2D points in image plane

    # Prepare object points
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            obj_points.append(objp)
            img_points.append(corners)

    # Perform calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None
    )

    return mtx, dist  # Camera matrix and distortion coefficients
```

### Extrinsic Calibration

Calibrating the relationship between different sensors:

```python
def calibrate_extrinsics(sensor1_points, sensor2_points):
    """Calibrate transformation between two sensors"""
    # Find transformation matrix between coordinate systems
    transformation, _ = cv2.findTransformECC(
        sensor1_points, sensor2_points,
        np.eye(4, dtype=np.float32)
    )

    return transformation
```

## Visual SLAM Fundamentals

### Key Components of VSLAM

VSLAM systems typically include:

1. **Feature Detection**: Identifying distinctive points in images
2. **Feature Matching**: Associating features across frames
3. **Pose Estimation**: Determining camera position and orientation
4. **Mapping**: Building a representation of the environment
5. **Loop Closure**: Recognizing previously visited locations

### ORB-SLAM Implementation

```python
import cv2
import numpy as np

class ORB_SLAM:
    def __init__(self):
        # Initialize ORB detector
        self.orb = cv2.ORB_create(nfeatures=2000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Initialize pose tracking
        self.current_pose = np.eye(4)
        self.keyframes = []
        self.map_points = []

    def process_frame(self, image):
        """Process a single frame for SLAM"""
        # Detect features
        kp, des = self.orb.detectAndCompute(image, None)

        if len(self.keyframes) == 0:
            # First frame - initialize
            self.keyframes.append({
                'image': image,
                'keypoints': kp,
                'descriptors': des,
                'pose': self.current_pose.copy()
            })
            return self.current_pose

        # Match with previous frame
        prev_frame = self.keyframes[-1]
        matches = self.bf.match(prev_frame['descriptors'], des)

        # Sort matches by distance
        matches = sorted(matches, key=lambda x: x.distance)

        # Extract matched points
        prev_pts = np.float32([prev_frame['keypoints'][m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        curr_pts = np.float32([kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Estimate motion using Essential matrix
        E, mask = cv2.findEssentialMat(curr_pts, prev_pts, focal=500, pp=(320, 240), method=cv2.RANSAC)

        if E is not None:
            # Recover pose
            _, R, t, mask_pose = cv2.recoverPose(E, curr_pts, prev_pts)

            # Update current pose
            transformation = np.eye(4)
            transformation[:3, :3] = R
            transformation[:3, 3] = t.flatten()

            self.current_pose = self.current_pose @ transformation

        # Add keyframe if significant movement
        if self.should_add_keyframe():
            self.keyframes.append({
                'image': image,
                'keypoints': kp,
                'descriptors': des,
                'pose': self.current_pose.copy()
            })

        return self.current_pose

    def should_add_keyframe(self):
        """Determine if a new keyframe should be added"""
        if len(self.keyframes) == 0:
            return True

        # Check if enough movement has occurred
        prev_pose = self.keyframes[-1]['pose']
        current_translation = np.linalg.norm(
            self.current_pose[:3, 3] - prev_pose[:3, 3]
        )

        return current_translation > 0.5  # Add keyframe if moved more than 0.5m
```

## Deep Learning-Based VSLAM

### CNN Feature Extraction

Using convolutional neural networks for feature extraction:

```python
import torch
import torch.nn as nn

class CNNFeatureExtractor(nn.Module):
    def __init__(self):
        super(CNNFeatureExtractor, self).__init__()

        # Feature extraction backbone
        self.backbone = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=7, stride=2, padding=3),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=3, stride=2, padding=1),
            nn.Conv2d(64, 128, kernel_size=5, stride=2, padding=2),
            nn.ReLU(),
            nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1),
            nn.ReLU()
        )

        # Feature descriptor head
        self.descriptor_head = nn.Sequential(
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
            nn.Linear(256, 128),
            nn.ReLU()
        )

    def forward(self, x):
        features = self.backbone(x)
        descriptors = self.descriptor_head(features)
        return descriptors

# Initialize the feature extractor
cnn_features = CNNFeatureExtractor()
```

### Semantic SLAM

Integrating semantic information into SLAM:

```python
class SemanticSLAM:
    def __init__(self):
        self.vslam = ORB_SLAM()
        self.segmentation_model = self.load_segmentation_model()

    def process_frame(self, image):
        """Process frame with both geometric and semantic information"""
        # Get geometric pose from VSLAM
        pose = self.vslam.process_frame(image)

        # Get semantic segmentation
        segmentation = self.segmentation_model(image)

        # Combine geometric and semantic information
        semantic_map = self.update_semantic_map(image, segmentation, pose)

        return pose, semantic_map

    def update_semantic_map(self, image, segmentation, pose):
        """Update semantic map with new information"""
        # Project semantic information into 3D space using pose
        projected_semantics = self.project_to_3d(segmentation, pose)

        # Update map with new semantic information
        self.integrate_semantic_info(projected_semantics, pose)

        return self.semantic_map
```

## Multi-Sensor Fusion

### IMU Integration

Integrating IMU data with visual odometry:

```python
class VisualInertialOdometry:
    def __init__(self):
        self.vslam = ORB_SLAM()
        self.imu_bias = np.zeros(6)  # 3 for accel, 3 for gyro

    def integrate_imu(self, imu_data, dt):
        """Integrate IMU measurements"""
        # Correct for bias
        corrected_gyro = imu_data['gyro'] - self.imu_bias[3:]
        corrected_accel = imu_data['accel'] - self.imu_bias[:3]

        # Integrate to get pose
        rotation = self.integrate_gyro(corrected_gyro, dt)
        velocity = self.integrate_accel(corrected_accel, dt)

        return rotation, velocity

    def fuse_visual_inertial(self, image, imu_data, dt):
        """Fuse visual and inertial measurements"""
        # Get visual pose
        visual_pose = self.vslam.process_frame(image)

        # Get inertial measurements
        rotation, velocity = self.integrate_imu(imu_data, dt)

        # Fuse measurements using Kalman filter or optimization
        fused_pose = self.kalman_filter.update(visual_pose, rotation, velocity)

        return fused_pose
```

### Sensor Fusion Architecture

```yaml
# Multi-sensor fusion configuration
sensor_fusion:
  cameras:
    - name: "front_camera"
      type: "rgb_depth"
      rate: 30.0
      topic: "/camera/rgb/image_raw"
    - name: "realsense"
      type: "stereo_depth"
      rate: 30.0
      topic: "/realsense/depth/image_rect_raw"

  imu:
    topic: "/imu/data"
    rate: 200.0

  odometry:
    topic: "/joint_states"
    rate: 100.0

  fusion_method: "ekf"  # Extended Kalman Filter
  update_rate: 100.0
```

## Real-Time Considerations

### Computational Optimization

Optimizing VSLAM for real-time performance:

```python
class OptimizedVSLAM:
    def __init__(self):
        # Use GPU acceleration where possible
        self.use_gpu = torch.cuda.is_available()

        # Multithreading for different components
        self.feature_thread = threading.Thread(target=self.extract_features)
        self.matching_thread = threading.Thread(target=self.match_features)
        self.pose_thread = threading.Thread(target=self.estimate_pose)

        # Image pyramid for efficient processing
        self.pyramid_levels = 3

    def extract_features_optimized(self, image):
        """Optimized feature extraction"""
        # Use image pyramid for faster processing
        pyramid = self.create_image_pyramid(image, self.pyramid_levels)

        all_features = []
        for level, img in enumerate(pyramid):
            # Extract features at different scales
            features = self.extract_at_scale(img, scale=2**level)
            all_features.extend(features)

        return all_features

    def create_image_pyramid(self, image, levels):
        """Create Gaussian pyramid for multi-scale processing"""
        pyramid = [image]
        for i in range(1, levels):
            pyramid.append(cv2.pyrDown(pyramid[-1]))
        return pyramid
```

### Memory Management

Efficient memory usage for continuous operation:

```python
class MemoryEfficientSLAM:
    def __init__(self, max_keyframes=100):
        self.max_keyframes = max_keyframes
        self.keyframe_buffer = collections.deque(maxlen=max_keyframes)

    def add_keyframe(self, keyframe):
        """Add keyframe with memory management"""
        if len(self.keyframe_buffer) >= self.max_keyframes:
            # Remove oldest keyframe
            oldest = self.keyframe_buffer.popleft()
            # Free memory associated with oldest keyframe
            self.free_keyframe_memory(oldest)

        self.keyframe_buffer.append(keyframe)

    def free_keyframe_memory(self, keyframe):
        """Free memory associated with keyframe"""
        # Clear descriptors and image data
        keyframe['descriptors'] = None
        keyframe['image'] = None
        keyframe['keypoints'] = None
```

## Performance Evaluation

### Accuracy Metrics

Evaluating VSLAM system performance:

1. **Absolute Trajectory Error (ATE)**: Difference between estimated and ground truth trajectory
2. **Relative Pose Error (RPE)**: Error in relative pose between consecutive poses
3. **Mapping accuracy**: How well the map matches the real environment

### Benchmarking

Using standard datasets for evaluation:

```python
def evaluate_vslam_on_dataset(vslam_system, dataset_path):
    """Evaluate VSLAM on standard dataset"""
    results = {
        'ate': [],
        'rpe': [],
        'processing_time': [],
        'tracking_success': 0
    }

    for frame in dataset:
        start_time = time.time()

        # Process frame
        pose = vslam_system.process_frame(frame['image'])

        processing_time = time.time() - start_time
        results['processing_time'].append(processing_time)

        # Compare with ground truth
        if frame['ground_truth'] is not None:
            ate = calculate_ate(pose, frame['ground_truth'])
            results['ate'].append(ate)

            results['tracking_success'] += 1

    return results
```

## Challenges in Humanoid Robotics

### Dynamic Body Movement

Humanoid robots present unique challenges:

- **Body motion compensation**: Accounting for robot's own movements
- **Occlusion handling**: Dealing with self-occlusion from robot's body
- **Motion blur**: Handling blur from rapid movements

### Computational Constraints

Running VSLAM on humanoid robots:

- **Power consumption**: Balancing accuracy with power usage
- **Thermal management**: Preventing overheating during intensive processing
- **Real-time requirements**: Meeting strict timing constraints

## Sensor Integration Code Examples

### 1. Multi-Sensor Fusion Node
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import cv2
from cv_bridge import CvBridge
import message_filters
from tf2_ros import TransformBroadcaster
import tf_transformations

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscribers for different sensors
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/rgb/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')

        # Synchronize messages from different sensors
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.imu_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sensor_callback)

        # Publisher for fused sensor data
        self.pose_pub = self.create_publisher(PoseStamped, '/fused_pose', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/fused_pointcloud', 10)

        self.get_logger().info('Sensor fusion node initialized')

    def sensor_callback(self, rgb_msg, depth_msg, imu_msg):
        """Process synchronized sensor data"""
        try:
            # Convert ROS images to OpenCV
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

            # Extract pose from IMU data
            imu_pose = self.extract_pose_from_imu(imu_msg)

            # Fuse visual and IMU data
            fused_pose = self.fuse_visual_imu(rgb_image, depth_image, imu_pose)

            # Publish fused pose
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose = fused_pose

            self.pose_pub.publish(pose_msg)

            # Generate point cloud from RGB-D data
            pointcloud = self.generate_pointcloud(rgb_image, depth_image)
            self.pointcloud_pub.publish(pointcloud)

        except Exception as e:
            self.get_logger().error(f'Error in sensor callback: {str(e)}')

    def extract_pose_from_imu(self, imu_msg):
        """Extract pose information from IMU data"""
        # Extract orientation from IMU quaternion
        orientation = [
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w
        ]

        # Convert quaternion to Euler angles for easier processing
        euler = tf_transformations.euler_from_quaternion(orientation)

        # For pose estimation, integrate angular velocities
        # This is simplified - in practice, you'd integrate over time
        pose = {
            'position': np.array([0.0, 0.0, 0.0]),  # Would need integration
            'orientation': orientation,
            'euler': euler
        }

        return pose

    def fuse_visual_imu(self, rgb_image, depth_image, imu_pose):
        """Fuse visual and IMU data for improved pose estimation"""
        # Extract visual features (ORB, SIFT, etc.)
        orb = cv2.ORB_create()
        keypoints, descriptors = orb.detectAndCompute(rgb_image, None)

        # Create a simple visual-inertial fusion
        # In practice, this would use more sophisticated algorithms
        # like Extended Kalman Filter or optimization-based methods

        # For now, we'll return a placeholder pose
        pose = PoseStamped()
        # This would be the result of fusing visual and IMU data
        return pose.pose

    def generate_pointcloud(self, rgb_image, depth_image):
        """Generate point cloud from RGB-D data"""
        # Get camera parameters (in practice, these would come from camera_info topic)
        fx, fy = 525.0, 525.0  # Focal lengths
        cx, cy = 319.5, 239.5  # Principal points (assuming 640x480 image)

        height, width = depth_image.shape
        points = []

        for v in range(height):
            for u in range(width):
                z = depth_image[v, u]
                if z > 0:  # Valid depth
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    r, g, b = rgb_image[v, u]
                    points.append([x, y, z, r, g, b])

        # Convert to PointCloud2 message (simplified)
        # In practice, you'd use sensor_msgs_py.point_cloud2
        pointcloud_msg = PointCloud2()
        return pointcloud_msg

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()

    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. RealSense D435i Integration with ROS 2
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Header
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create publishers
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/camera/imu/data', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', 10)

        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable streams
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
        self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

        # Start pipeline
        self.pipeline.start(self.config)

        # Timer for publishing data
        self.timer = self.create_timer(0.033, self.publish_sensor_data)  # ~30 Hz

        self.get_logger().info('RealSense D435i node initialized')

    def publish_sensor_data(self):
        """Publish RealSense sensor data to ROS topics"""
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)

            # Get RGB frame
            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())

                # Convert to ROS Image message
                rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                rgb_msg.header.stamp = self.get_clock().now().to_msg()
                rgb_msg.header.frame_id = "camera_color_optical_frame"

                self.rgb_pub.publish(rgb_msg)

                # Publish camera info
                camera_info_msg = self.create_camera_info_msg(color_frame)
                camera_info_msg.header = rgb_msg.header
                self.camera_info_pub.publish(camera_info_msg)

            # Get depth frame
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())

                # Convert to ROS Image message
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
                depth_msg.header.stamp = self.get_clock().now().to_msg()
                depth_msg.header.frame_id = "camera_depth_optical_frame"

                self.depth_pub.publish(depth_msg)

            # Get IMU data (if available)
            accel_frame = frames.first_or_default(rs.stream.accel, rs.frame())
            gyro_frame = frames.first_or_default(rs.stream.gyro, rs.frame())

            if accel_frame and gyro_frame:
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()

                # Create IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "camera_imu_frame"

                # Set angular velocity (gyro data)
                imu_msg.angular_velocity.x = gyro_data.x
                imu_msg.angular_velocity.y = gyro_data.y
                imu_msg.angular_velocity.z = gyro_data.z

                # Set linear acceleration (accelerometer data)
                imu_msg.linear_acceleration.x = accel_data.x
                imu_msg.linear_acceleration.y = accel_data.y
                imu_msg.linear_acceleration.z = accel_data.z

                # Note: RealSense doesn't provide orientation directly
                # This would need to be computed from accelerometer/gyro data
                imu_msg.orientation_covariance[0] = -1  # Indicates no orientation data

                self.imu_pub.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing sensor data: {str(e)}')

    def create_camera_info_msg(self, color_frame):
        """Create camera info message from RealSense frame"""
        from sensor_msgs.msg import CameraInfo

        camera_info = CameraInfo()

        # Set camera parameters (these should match your RealSense calibration)
        camera_info.width = color_frame.get_width()
        camera_info.height = color_frame.get_height()

        # Default intrinsics (would need actual calibration values)
        camera_info.k = [525.0, 0.0, 319.5,
                         0.0, 525.0, 239.5,
                         0.0, 0.0, 1.0]

        camera_info.p = [525.0, 0.0, 319.5, 0.0,
                         0.0, 525.0, 239.5, 0.0,
                         0.0, 0.0, 1.0, 0.0]

        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion

        return camera_info

def main(args=None):
    rclpy.init(args=args)
    realsense_node = RealSenseNode()

    try:
        rclpy.spin(realsense_node)
    except KeyboardInterrupt:
        pass
    finally:
        realsense_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Sensor Calibration Pipeline
```python
import numpy as np
import cv2
import glob
import yaml
from scipy.spatial.transform import Rotation as R

class SensorCalibration:
    def __init__(self):
        # Chessboard dimensions
        self.chessboard_size = (9, 6)  # 9x6 internal corners
        self.square_size = 0.025  # 2.5 cm squares

        # Prepare object points
        self.objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2) * self.square_size

    def calibrate_camera(self, images_path_pattern):
        """Calibrate a single camera using chessboard images"""
        # Arrays to store object points and image points
        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane

        # Get list of image files
        images = glob.glob(images_path_pattern)

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

            if ret:
                objpoints.append(self.objp)

                # Refine corner locations
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

        if len(objpoints) > 0:
            # Perform camera calibration
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )

            return {
                'camera_matrix': mtx,
                'distortion_coefficients': dist,
                'rotation_vectors': rvecs,
                'translation_vectors': tvecs,
                'reprojection_error': ret
            }
        else:
            return None

    def calibrate_rgb_depth(self, rgb_images_pattern, depth_images_pattern):
        """Calibrate RGB and depth cameras for an RGB-D sensor"""
        # Calibrate individual cameras
        rgb_calib = self.calibrate_camera(rgb_images_pattern)
        depth_calib = self.calibrate_camera(depth_images_pattern)

        # Find corresponding chessboard detections in both cameras
        rgb_images = glob.glob(rgb_images_pattern)
        depth_images = glob.glob(depth_images_pattern)

        # For extrinsic calibration, we need to detect the same pattern
        # in both cameras and compute the transformation between them
        common_corners = []

        for rgb_img_path, depth_img_path in zip(rgb_images, depth_images):
            rgb_img = cv2.imread(rgb_img_path)
            depth_img = cv2.imread(depth_img_path)

            gray_rgb = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
            gray_depth = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)

            ret_rgb, corners_rgb = cv2.findChessboardCorners(
                gray_rgb, self.chessboard_size, None
            )
            ret_depth, corners_depth = cv2.findChessboardCorners(
                gray_depth, self.chessboard_size, None
            )

            if ret_rgb and ret_depth:
                # Refine corner locations
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_rgb = cv2.cornerSubPix(
                    gray_rgb, corners_rgb, (11, 11), (-1, -1), criteria
                )
                corners_depth = cv2.cornerSubPix(
                    gray_depth, corners_depth, (11, 11), (-1, -1), criteria
                )

                common_corners.append((corners_rgb, corners_depth))

        if len(common_corners) >= 3:  # Need at least 3 pairs for calibration
            # Compute extrinsic transformation
            rvec, tvec = self.compute_extrinsics(common_corners, rgb_calib, depth_calib)

            # Convert rotation vector to matrix
            rmat, _ = cv2.Rodrigues(rvec)

            return {
                'rgb_calibration': rgb_calib,
                'depth_calibration': depth_calib,
                'rotation_matrix': rmat,
                'translation_vector': tvec,
                'transformation_matrix': np.hstack([rmat, tvec.reshape(3, 1)])
            }

        return None

    def compute_extrinsics(self, common_corners, rgb_calib, depth_calib):
        """Compute extrinsic parameters between RGB and depth cameras"""
        # This is a simplified approach
        # In practice, you'd use more sophisticated methods

        # Get the first valid pair
        rgb_corners, depth_corners = common_corners[0]

        # Use solvePnP to find the transformation
        # This assumes we have the object points and image points
        obj_points = self.objp
        img_points = depth_corners.reshape(-1, 2)

        # Use the depth camera's intrinsic parameters
        camera_matrix = depth_calib['camera_matrix']
        dist_coeffs = depth_calib['distortion_coefficients']

        ret, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, camera_matrix, dist_coeffs
        )

        return rvec, tvec

# Usage example
def main():
    calibrator = SensorCalibration()

    # Calibrate RGB-D camera
    calibration_result = calibrator.calibrate_rgb_depth(
        "calibration_images/rgb/*.jpg",
        "calibration_images/depth/*.png"
    )

    if calibration_result:
        print("Calibration successful!")
        print(f"RGB Camera Matrix:\n{calibration_result['rgb_calibration']['camera_matrix']}")
        print(f"Depth Camera Matrix:\n{calibration_result['depth_calibration']['camera_matrix']}")
        print(f"Extrinsic Rotation:\n{calibration_result['rotation_matrix']}")
        print(f"Extrinsic Translation:\n{calibration_result['translation_vector']}")

        # Save calibration to file
        with open('sensor_calibration.yaml', 'w') as f:
            yaml.dump(calibration_result, f)
    else:
        print("Calibration failed!")

if __name__ == "__main__":
    main()
```

### 4. Sensor Data Processing Pipeline
```python
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import threading
import queue
import time

class SensorDataProcessor:
    def __init__(self):
        # Queues for sensor data
        self.rgb_queue = queue.Queue(maxsize=10)
        self.depth_queue = queue.Queue(maxsize=10)
        self.imu_queue = queue.Queue(maxsize=10)

        # Processing flags
        self.is_processing = False

        # Sensor calibration parameters (these would be loaded from calibration file)
        self.rgb_intrinsics = np.array([
            [525.0, 0.0, 319.5],
            [0.0, 525.0, 239.5],
            [0.0, 0.0, 1.0]
        ])

        self.depth_intrinsics = np.array([
            [525.0, 0.0, 319.5],
            [0.0, 525.0, 239.5],
            [0.0, 0.0, 1.0]
        ])

        # Extrinsic transformation between RGB and depth
        self.extrinsics = np.eye(4)  # Identity matrix (no transformation)

    def start_processing(self):
        """Start the sensor data processing pipeline"""
        self.is_processing = True

        # Start processing threads
        self.processing_thread = threading.Thread(target=self._process_loop)
        self.processing_thread.start()

        print("Sensor data processing started")

    def stop_processing(self):
        """Stop the sensor data processing pipeline"""
        self.is_processing = False
        if hasattr(self, 'processing_thread'):
            self.processing_thread.join()

        print("Sensor data processing stopped")

    def add_rgb_data(self, image, timestamp):
        """Add RGB image data to the processing queue"""
        try:
            self.rgb_queue.put((image, timestamp), block=False)
        except queue.Full:
            print("RGB queue is full, dropping frame")

    def add_depth_data(self, depth_image, timestamp):
        """Add depth image data to the processing queue"""
        try:
            self.depth_queue.put((depth_image, timestamp), block=False)
        except queue.Full:
            print("Depth queue is full, dropping frame")

    def add_imu_data(self, linear_accel, angular_vel, timestamp):
        """Add IMU data to the processing queue"""
        try:
            self.imu_queue.put((linear_accel, angular_vel, timestamp), block=False)
        except queue.Full:
            print("IMU queue is full, dropping data")

    def _process_loop(self):
        """Main processing loop"""
        while self.is_processing:
            # Process synchronized sensor data
            self._process_synchronized_data()
            time.sleep(0.01)  # 10ms sleep to prevent busy waiting

    def _process_synchronized_data(self):
        """Process synchronized sensor data"""
        # Try to get synchronized data from queues
        try:
            # Get latest RGB frame
            rgb_frame, rgb_ts = self.rgb_queue.get_nowait()

            # Get latest depth frame
            depth_frame, depth_ts = self.depth_queue.get_nowait()

            # Get latest IMU data
            linear_accel, angular_vel, imu_ts = self.imu_queue.get_nowait()

            # Synchronize timestamps (simplified approach)
            # In practice, you'd use more sophisticated time synchronization
            time_diff = abs(rgb_ts - depth_ts)
            if time_diff > 0.05:  # 50ms threshold
                print(f"Warning: Large time difference between RGB and depth: {time_diff}s")

            # Process the data
            processed_data = self._process_frame_pair(rgb_frame, depth_frame, linear_accel, angular_vel)

            # Publish results (placeholder)
            self._publish_results(processed_data)

        except queue.Empty:
            # No synchronized data available, continue
            pass

    def _process_frame_pair(self, rgb_image, depth_image, linear_accel, angular_vel):
        """Process synchronized RGB and depth frames with IMU data"""
        # Convert depth to point cloud
        point_cloud = self._depth_to_pointcloud(depth_image, self.depth_intrinsics)

        # Apply extrinsic transformation if needed
        transformed_points = self._apply_extrinsics(point_cloud, self.extrinsics)

        # Fuse with IMU data for improved pose estimation
        fused_pose = self._fuse_with_imu(transformed_points, linear_accel, angular_vel)

        # Perform feature extraction on RGB image
        features = self._extract_features(rgb_image)

        # Combine all processed data
        result = {
            'point_cloud': transformed_points,
            'fused_pose': fused_pose,
            'features': features,
            'timestamp': time.time()
        }

        return result

    def _depth_to_pointcloud(self, depth_image, intrinsics):
        """Convert depth image to 3D point cloud"""
        height, width = depth_image.shape

        # Create coordinate grids
        u, v = np.meshgrid(np.arange(width), np.arange(height))

        # Flatten arrays
        u_flat = u.flatten()
        v_flat = v.flatten()
        depth_flat = depth_image.flatten()

        # Filter out invalid depth values
        valid_mask = depth_flat > 0
        u_valid = u_flat[valid_mask]
        v_valid = v_flat[valid_mask]
        z_valid = depth_flat[valid_mask]

        # Apply inverse intrinsic transformation
        fx, fy = intrinsics[0, 0], intrinsics[1, 1]
        cx, cy = intrinsics[0, 2], intrinsics[1, 2]

        x = (u_valid - cx) * z_valid / fx
        y = (v_valid - cy) * z_valid / fy

        # Create point cloud
        points = np.column_stack([x, y, z_valid])

        return points

    def _apply_extrinsics(self, points, extrinsics):
        """Apply extrinsic transformation to points"""
        # Convert to homogeneous coordinates
        points_homo = np.hstack([points, np.ones((points.shape[0], 1))])

        # Apply transformation
        transformed_points_homo = points_homo @ extrinsics.T

        # Convert back to 3D
        transformed_points = transformed_points_homo[:, :3]

        return transformed_points

    def _fuse_with_imu(self, points, linear_accel, angular_vel):
        """Fuse point cloud data with IMU for pose estimation"""
        # This is a simplified approach
        # In practice, you'd use more sophisticated fusion algorithms

        # Integrate angular velocity to get orientation
        # This would require integration over time in a real system
        dt = 0.01  # 10ms time step

        # Simple integration (in practice, use proper integration techniques)
        orientation = R.from_rotvec(angular_vel * dt)

        # For now, return a placeholder pose
        pose = {
            'position': np.array([0.0, 0.0, 0.0]),  # Would need integration
            'orientation': orientation.as_quat(),   # [x, y, z, w]
            'linear_acceleration': linear_accel,
            'angular_velocity': angular_vel
        }

        return pose

    def _extract_features(self, image):
        """Extract features from RGB image"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Extract ORB features
        orb = cv2.ORB_create(nfeatures=1000)
        keypoints, descriptors = orb.detectAndCompute(gray, None)

        # Convert to usable format
        features = {
            'keypoints': [(kp.pt[0], kp.pt[1]) for kp in keypoints] if keypoints else [],
            'descriptors': descriptors,
            'count': len(keypoints) if keypoints else 0
        }

        return features

    def _publish_results(self, processed_data):
        """Publish processed sensor data (placeholder)"""
        # In a real system, this would publish to ROS topics or other interfaces
        print(f"Processed frame with {len(processed_data['point_cloud'])} points, "
              f"{processed_data['features']['count']} features")

# Example usage
def example_usage():
    processor = SensorDataProcessor()
    processor.start_processing()

    # Simulate adding sensor data (in a real system, this would come from actual sensors)
    # This is just a conceptual example
    import time

    # Stop after 5 seconds for demo purposes
    time.sleep(5)
    processor.stop_processing()

if __name__ == "__main__":
    example_usage()
```

### Sensor Integration Best Practices

When integrating sensors for humanoid robotics:

1. **Time Synchronization**: Ensure all sensors are properly synchronized using hardware triggers or software timestamping
2. **Calibration**: Perform regular calibration of intrinsic and extrinsic parameters
3. **Data Validation**: Implement checks to validate sensor data quality before processing
4. **Resource Management**: Monitor computational and memory usage to avoid bottlenecks
5. **Failure Handling**: Implement fallback mechanisms when sensors fail or provide poor data
6. **Data Association**: Properly associate data from different sensors using timestamps and spatial relationships
7. **Filtering**: Apply appropriate filtering to reduce noise while preserving important information

## Summary

Vision sensors and VSLAM form the eyes of humanoid robots, enabling them to perceive and understand their environment. By combining traditional computer vision techniques with modern deep learning approaches and proper sensor fusion, humanoid robots can achieve robust localization and mapping capabilities. The key to success lies in proper calibration, efficient algorithms, and careful consideration of the computational constraints of humanoid platforms.