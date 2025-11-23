#include "rclcpp/rclcpp.hpp"

// 3.创建类对象
class MyParam : public rclcpp::Node{
public:
  // 构造函数
  MyParam() : Node("MyParam"){
    // 创建参数类对象
    rclcpp::Parameter p1("car_name","BWM");
    rclcpp::Parameter p2("car_height",1.68);
    rclcpp::Parameter p3("car_wheels",4);

    // 访问参数的值
    RCLCPP_INFO(this->get_logger(),"car_name = %s",p1.as_string().c_str());
    RCLCPP_INFO(this->get_logger(),"car_height = %.2f",p2.as_double());
    RCLCPP_INFO(this->get_logger(),"car_wheels = %d",p3.as_int());
    
    // 访问参数的键
    RCLCPP_INFO(this->get_logger(),"p1_key = %s",p1.get_name().c_str());
    RCLCPP_INFO(this->get_logger(),"p2_key = %s",p2.get_name().c_str());
    RCLCPP_INFO(this->get_logger(),"p3_key = %s",p3.get_name().c_str());
    
    // 访问参数值的数据类型
    RCLCPP_INFO(this->get_logger(),"p1_typename = %s",p1.get_type_name().c_str());
    RCLCPP_INFO(this->get_logger(),"p2_typename = %s",p2.get_type_name().c_str());
    RCLCPP_INFO(this->get_logger(),"p3_typename = %s",p3.get_type_name().c_str());

  }
};

int main(int argc, char const *argv[])
{
  // 1.节点初始化
  rclcpp::init(argc,argv);
  // 2.spin函数
  rclcpp::spin(std::make_shared<MyParam>());
  // 4.释放资源
  rclcpp::shutdown();  
  return 0;
}

