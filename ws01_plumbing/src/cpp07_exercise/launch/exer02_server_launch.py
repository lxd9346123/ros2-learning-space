from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1.启动乌龟GUI节点
    t1 = Node(package="turtlesim", executable="turtlesim_node")

    # 2.启动服务端节点
    server = Node(package="cpp07_exercise",executable="exer02_server")

    return LaunchDescription([t1,server])