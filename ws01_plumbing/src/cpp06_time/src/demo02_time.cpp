#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 3.类声明
class MyTimer : public rclcpp::Node{
public:
  MyTimer() : Node("MyTimer"){
    demo_time();
  }
private:
  void demo_time(){
    rclcpp::Time t1(500000000L);
    rclcpp::Time t2(2,500000000L);
    auto t3 = this->now();

    RCLCPP_INFO(this->get_logger(),"t1:s = %.2f, ns = %ld",t1.seconds(),t1.nanoseconds());
    RCLCPP_INFO(this->get_logger(),"t2:s = %.2f, ns = %ld",t2.seconds(),t2.nanoseconds());
    RCLCPP_INFO(this->get_logger(),"t3:s = %.2f, ns = %ld",t3.seconds(),t3.nanoseconds());
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
