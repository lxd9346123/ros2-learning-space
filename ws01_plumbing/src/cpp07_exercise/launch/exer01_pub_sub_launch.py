from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 启动两个乌龟节点
    turtle1 = Node(package="turtlesim",executable="turtlesim_node")
    turtle2 = Node(package="turtlesim",executable="turtlesim_node",namespace="t2")

    # 使第二只乌龟旋转的命令
    rotate = ExecuteProcess(
        cmd=["ros2 action send_goal /t2/turtle1/rotate_absolute turtlesim/action/RotateAbsolute \"{'theta': 3.14}\""],
        shell=True,
        output="both",
    )
    # 启动自定义的发布订阅节点
    execute_process = Node(package="cpp07_exercise", executable="exer01_pub_sub")

    register_rotate_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rotate,       # 目标节点
            on_exit=execute_process    # 退出后启动的节点
        )
    )

    return LaunchDescription([turtle1, turtle2, rotate,register_rotate_exit_handler])