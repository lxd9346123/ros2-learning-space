from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package = "turtlesim", executable = "turtlesim_node", name = "t1",namespace = "myns",
             remappings=[("/myns/turtle1/cmd_vel","/cmd_vel")])
    ])