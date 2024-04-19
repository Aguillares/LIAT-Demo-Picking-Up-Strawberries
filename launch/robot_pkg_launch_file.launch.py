from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    camera_node = Node (
            package = 'robot_pkg',
            executable = 'camera',
            output = 'screen',
    )
    robot_node = Node (
            package = 'robot_pkg',
            executable = 'robot',
            output = 'screen',
    )
    rqt_image_view_node = ExecuteProcess(
        	cmd=['ros2','run','rqt_image_view','rqt_image_view','--','-t', 'realsense/image'],
        	output='screen'
    )

    return LaunchDescription([
    	camera_node,
    	robot_node,
    	rqt_image_view_node       
    	])

