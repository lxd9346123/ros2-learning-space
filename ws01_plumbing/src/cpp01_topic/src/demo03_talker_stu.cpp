#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using namespace std::chrono_literals;
using namespace base_interfaces_demo::msg;

// 3.创建发布者类
class TalkerStu : public rclcpp::Node{
public:
    // 构造函数
    TalkerStu() : Node("TalkerStu"){
        publisher_ = this->create_publisher<Student>("student_info",10);
        timer_ = this->create_wall_timer(500ms,std::bind(&TalkerStu::timer_callback,this));
    }
private:
    // 定时回调函数
    void timer_callback(){
        auto message = Student();
        age_ ++;
        message.name = "huluwa";
        message.age = age_;
        message.height = 2.20;

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(),"发布的消息是:(%s,%d,%.2f)",message.name.c_str(),message.age,message.height);
    }
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t age_;
};

int main(int argc, char const *argv[])
{
    // 1.节点初始化
    rclcpp::init(argc,argv);
    // 2.回调函数
    rclcpp::spin(std::make_shared<TalkerStu>());
    // 4.释放资源
    rclcpp::shutdown();
    return 0;
}
