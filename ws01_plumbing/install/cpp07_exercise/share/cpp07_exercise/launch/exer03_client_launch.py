from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 1.新生成一只乌龟
    x = 6.0
    y = 9.0
    theta = 0.0

    newturtle = ExecuteProcess(
        cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{'x':" + 
            str(x) + ",'y':" + str(y) +",'theta':"+ str(theta) +"}\""],
        shell=True,
        output="both"
    )

    # 2.启动客户端节点
    client = Node(package="cpp07_exercise",
                  executable="exer03_client",
                  arguments=[str(x),str(y),str(theta)])

    return LaunchDescription([newturtle,client])