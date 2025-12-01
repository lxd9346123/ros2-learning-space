#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 3.类声明
class MyNames : public rclcpp::Node{
public:
  // 设置节点的命名空间
  MyNames() : Node("MyTimer","t1_ns"){
    // 全局话题(与命令空间同级)
    // pub_ = this->create_publisher<std_msgs::msg::String>("/shi",10);
    // 相对话题(在命名空间里面)
    // pub_ = this->create_publisher<std_msgs::msg::String>("shi",10);
    // 私有话题(在节点名称里面)
    pub_ = this->create_publisher<std_msgs::msg::String>("~/shi",10);
  }
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char const *argv[])
{
  // 1.初始化节点
  rclcpp::init(argc,argv);
  // 2.spin函数
  rclcpp::spin(std::make_shared<MyNames>());
  // 4.释放资源
  rclcpp::shutdown();
  return 0;
}
