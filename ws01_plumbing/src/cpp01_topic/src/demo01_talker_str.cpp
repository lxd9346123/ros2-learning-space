// 1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// 3.自定义节点类
class Talker: public rclcpp::Node{
public:
  Talker():Node("talker_node_cpp"),count(0){
    RCLCPP_INFO(this->get_logger(),"发布节点创建！");
    // 3-1.创建消息发布方
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter",10);
    // 3-2.创建定时器
    timer_ = this->create_wall_timer(1s,std::bind(&Talker::on_timer,this));
  }
private:
  // 3-3.组织并且发布消息 
  void on_timer(){
    auto message = std_msgs::msg::String();
    message.data = "hello world!" + std::to_string(count++);
    RCLCPP_INFO(this->get_logger(),"发布方发布的消息:%s",message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count;
};

int main(int argc, char const *argv[])
{
  // 2.初始化节点
  rclcpp::init(argc,argv);
  // 4.调用spin函数，传入自定义类对象指针
  rclcpp::spin(std::make_shared<Talker>());
  // 5.释放资源
  rclcpp::shutdown();
  return 0;
}

