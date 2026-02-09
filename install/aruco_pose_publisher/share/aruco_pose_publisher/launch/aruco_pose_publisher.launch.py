from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    video_url = LaunchConfiguration('video_url', default='http://10.152.70.62/video_raw/tracking_down')
    marker_size = LaunchConfiguration('marker_size', default='0.112')
    intrinsics_path = LaunchConfiguration('intrinsics_path', default='')
    frame_id = LaunchConfiguration('frame_id', default='camera_frame')

    return LaunchDescription([
        DeclareLaunchArgument('video_url', default_value=video_url,
                              description='Video stream URL or device index'),
        DeclareLaunchArgument('marker_size', default_value=marker_size,
                              description='Marker size in meters'),
        DeclareLaunchArgument('intrinsics_path', default_value=intrinsics_path,
                              description='Path to OpenCV YAML intrinsics file'),
        DeclareLaunchArgument('frame_id', default_value=frame_id,
                              description='Frame ID for Pose messages'),
        Node(
            package='aruco_pose_publisher',
            executable='aruco_pose_publisher_node',
            name='aruco_pose_publisher',
            output='screen',
            parameters=[{
                'video_url': video_url,
                'marker_size': marker_size,
                'intrinsics_path': intrinsics_path,
                'frame_id': frame_id,
            }]
        )
    ])
