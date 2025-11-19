#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

// 3.订阅类实现
class Listener : public rclcpp::Node{
public:
    // 构造函数
    Listener() : Node("listener_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"订阅者节点创建！");
        listener_ = this->create_subscription<std_msgs::msg::String>("chatter",10,std::bind(&Listener::do_cb,this,_1));
    }
private:
    // 回调函数
    void do_cb(const std_msgs::msg::String &msg) const{
        RCLCPP_INFO(this->get_logger(),"订阅到的数据：%s",msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listener_;
};

int main(int argc, char const *argv[])
{
    // 1.节点初始化
    rclcpp::init(argc,argv);
    // 2.spin函数
    rclcpp::spin(std::make_shared<Listener>());
    // 4.释放资源
    rclcpp::shutdown();
    return 0;
}
