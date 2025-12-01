#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 3.类声明
class MyTimer : public rclcpp::Node{
public:
  MyTimer() : Node("MyTimer"){
    demo_rate();
  }
private:
  void demo_rate(){
    rclcpp::Rate rate1(500ms);    // 按照时间休眠(500ms)
    rclcpp::Rate rate2(1.0);      // 按照频率休眠(1Hz)

    while(rclcpp::ok()){
      RCLCPP_INFO(this->get_logger(),"hello");
      // rate1.sleep();
      rate2.sleep();
    }
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
