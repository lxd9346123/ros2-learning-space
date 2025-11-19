#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using std::placeholders::_1;
using namespace base_interfaces_demo::msg;

// 3.订阅者类创建
class ListenerStu : public rclcpp::Node{
public:
    // 构造函数
    ListenerStu(): Node("ListenerStu"){
        RCLCPP_INFO(this->get_logger(),"订阅者节点创建!");
        listener_ = this->create_subscription<Student>("student_info",10,std::bind(&ListenerStu::topic_callback,this,_1));
    }
private:
    // 订阅话题回调函数
    void topic_callback(const Student &msg){
        RCLCPP_INFO(this->get_logger(),"订阅到的数据为:(%s,%d,%.2f)",msg.name.c_str(),msg.age,msg.height);
    }
    rclcpp::Subscription<Student>::SharedPtr listener_;
};

int main(int argc, char const *argv[])
{
    // 1.节点初始化
    rclcpp::init(argc,argv);
    // 2.spin函数
    rclcpp::spin(std::make_shared<ListenerStu>());
    // 4.释放资源
    rclcpp::shutdown();
    return 0;
}
