#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using geometry_msgs::msg::Twist;
using turtlesim::msg::Pose;

class MinimalPubSub : public rclcpp::Node{
public:
  MinimalPubSub() : Node("MinimalPubSub"){
    // 提示输出
    RCLCPP_INFO(this->get_logger(),"Node has been started.");

    // 订阅第一只乌龟的位姿信息
    subscription_ = this->create_subscription<Pose>("/turtle1/pose",10,
      std::bind(&MinimalPubSub::topic_callback,this,std::placeholders::_1));

    // 创建发布第二只乌龟的控制指令
    publisher_ = this->create_publisher<Twist>("/t2/turtle1/cmd_vel",10);
  }

private:
  // 订阅回调函数
  void topic_callback(Pose::ConstSharedPtr pose){
    Twist twist1;
    twist1.linear.x = pose->linear_velocity;       // 线速度不变
    twist1.angular.z = - pose->angular_velocity;   // 角速度取反
    publisher_->publish(twist1);                   // 发送控制指令
  }

  rclcpp::Publisher<Twist>::SharedPtr publisher_;
  rclcpp::Subscription<Pose>::SharedPtr subscription_;
};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MinimalPubSub>());
  rclcpp::shutdown();
  return 0;
}
