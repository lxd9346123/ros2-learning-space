#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 3.类声明
class MyTimer : public rclcpp::Node{
public:
  MyTimer() : Node("MyTimer"){
    demo_duration();
  }
private:
  void demo_duration(){
    rclcpp::Duration du1(500ms);
    rclcpp::Duration du2(2,500000000L);

    RCLCPP_INFO(this->get_logger(),"du1: s = %.2f, ns = %ld",du1.seconds(),du1.nanoseconds());
    RCLCPP_INFO(this->get_logger(),"du2: s = %.2f, ns = %ld",du2.seconds(),du2.nanoseconds());
  }
};

int main(int argc, char const *argv[])
{
  // 1.初始化节点
  rclcpp::init(argc,argv);
  // 2.spin函数
  rclcpp::spin(std::make_shared<MyTimer>());
  // 4.释放资源
  rclcpp::shutdown();
  return 0;
}
