
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
   publish_tag_detections_image_launch_arg = DeclareLaunchArgument('publish_tag_detections_image', default_value=TextSubstitution(text='True'))


   return LaunchDescription([
      publish_tag_detections_image_launch_arg,
      Node(
         package='apriltag_ros2',
         executable='continuous_detector',
         name='continuous_detector',
         parameters=[{
            'publish_tag_detections_image': LaunchConfiguration('publish_tag_detections_image'),
         }]
      ),
   ])