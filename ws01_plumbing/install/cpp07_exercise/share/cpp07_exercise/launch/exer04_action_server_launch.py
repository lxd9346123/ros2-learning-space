from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 乌龟GUI节点
    turtle = Node(package="turtlesim",executable="turtlesim_node")
    # 自定义服务端节点
    server = Node(package="cpp07_exercise",executable="exer04_action_server")
    return LaunchDescription([turtle,server])