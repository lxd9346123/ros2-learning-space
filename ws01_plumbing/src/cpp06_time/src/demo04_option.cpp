#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 3.类声明
class MyTimer : public rclcpp::Node{
public:
  MyTimer() : Node("MyTimer"){
    demo_opt();
  }
private:
  // 演示运算符的使用
  void demo_opt(){
    rclcpp::Time t1(10,0);
    rclcpp::Time t2(30,0);

    rclcpp::Duration du1(8,0);
    rclcpp::Duration du2(17,0);

    // 运算
    // 比较运算
    RCLCPP_INFO(this->get_logger(),"t1 >= t2 ?  %d",t1 >= t2);
    RCLCPP_INFO(this->get_logger(),"t1 < t2 ?  %d",t1 < t2);
    // 数学运算
    rclcpp::Duration du3 = t2 - t1;
    rclcpp::Time t3 = t1 + du1;
    rclcpp::Time t4 = t1 - du1;
    RCLCPP_INFO(this->get_logger(),"du3 = %.2f",du3.seconds());
    RCLCPP_INFO(this->get_logger(),"t3 = %.2f",t3.seconds());
    RCLCPP_INFO(this->get_logger(),"t4 = %.2f",t4.seconds());
    
    RCLCPP_INFO(this->get_logger(),"du1 >= du2 ?  %d",du1 >= du2);
    RCLCPP_INFO(this->get_logger(),"du1 < du2 ?  %d",du1 < du2);
    rclcpp::Duration du4 = du1 * 3;
    rclcpp::Duration du5 = du1 + du2;
    rclcpp::Duration du6 = du1 - du2;
    RCLCPP_INFO(this->get_logger(),"du4 = %.2f",du4.seconds());
    RCLCPP_INFO(this->get_logger(),"du5 = %.2f",du5.seconds());
    RCLCPP_INFO(this->get_logger(),"du6 = %.2f",du6.seconds());

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
