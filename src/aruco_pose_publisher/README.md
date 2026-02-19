# aruco_pose_publisher

ROS 2 (Humble) C++ node that detects ArUco markers from a video stream and publishes their poses.

- Publishes geometry_msgs/PoseArray on `aruco/poses` (one pose per detected marker)
- Publishes geometry_msgs/PoseStamped for the first detected marker on `aruco/first_pose`
- Publishes std_msgs/Int32MultiArray of marker IDs on `aruco/ids`

## Parameters

- `video_url` (string): Video source. Default: `http://10.152.70.62/video_raw/tracking_down`
- `marker_size` (double): Marker size in meters. Default: `0.112`
- `intrinsics_path` (string): Path to an OpenCV YAML intrinsics file. If empty, pose estimation is disabled.
  The loader supports keys: `camera_matrix|K|M`, `distortion_coefficients|distCoeffs|D`, and optional `distortion_model` (set to `fisheye` for fisheye cameras).
- `frame_id` (string): Frame ID for messages. Default: `camera_frame`

You can reuse the intrinsics already in this repository, e.g.:

- `src/aruco_tracking/aruco_tracking/opencv_tracking_down_intrinsics.yml`
- `src/aruco_tracking/aruco_tracking/opencv_tracking_front_intrinsics.yml`

## Build

From the workspace root:

```bash
colcon build --packages-select aruco_pose_publisher
source install/setup.bash
```

## Run

Launch with defaults:

```bash
ros2 launch aruco_pose_publisher aruco_pose_publisher.launch.py \
  intrinsics_path:="$(pwd)/src/opencv_tracking_down_intrinsics.yml"
```

Or run the executable directly:

```bash
ros2 run aruco_pose_publisher aruco_pose_publisher_node \
  --ros-args -p video_url:="http://10.152.70.62/video_raw/tracking_down" \
             -p marker_size:=0.112 \
             -p intrinsics_path:="$(pwd)/src/aruco_tracking/aruco_tracking/opencv_tracking_down_intrinsics.yml" \
             -p frame_id:="camera_frame"
```

## Notes

- Fisheye cameras: set `distortion_model: fisheye` in your YAML intrinsics. The node will undistort points with `cv::fisheye::undistortPoints` and use IPPE for pose.
- If no intrinsics are provided, detection still runs but no poses are published (PoseArray will be empty).
