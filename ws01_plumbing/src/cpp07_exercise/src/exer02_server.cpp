#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

using turtlesim::msg::Pose;
using base_interfaces_demo::srv::Distance;
using std::placeholders::_1;
using std::placeholders::_2;

class exer_server : public rclcpp::Node{
public:
    exer_server() : Node("exer_server"){
        RCLCPP_INFO(this->get_logger(),"案例2服务通信服务端已创建!");
        subscription_ = this->create_subscription<Pose>("/turtle1/pose",10,
            std::bind(&exer_server::topic_callback,this,std::placeholders::_1));
        server_ = this->create_service<Distance>("/calculate",std::bind(&exer_server::service_callback,this,_1,_2));
    }
    
    
private:
    // 话题回调函数(接收原点乌龟的位置)
    void topic_callback(Pose::ConstSharedPtr pose){
        odomx_ = pose->x;
        odomy_ = pose->y;
    }
    // 接收客户端发送的坐标并且返回距离值
    void service_callback(Distance::Request::SharedPtr request,Distance::Response::SharedPtr response){
        float x = request->x;
        float y = request->y;
        float distance = sqrt((x - odomx_) * (x - odomx_)+(y - odomy_) * (y - odomy_));
        response->distance = distance;
        RCLCPP_INFO(this->get_logger(),"生成乌龟坐标:(%.2f,%.2f),原生乌龟坐标:(%.2f,%.2f),二者之间的距离:%.2f",
                    x,y,odomx_,odomy_,distance);
    }
    rclcpp::Subscription<Pose>::SharedPtr subscription_;
    rclcpp::Service<Distance>::SharedPtr  server_;
    float odomx_;
    float odomy_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<exer_server>());
    rclcpp::shutdown();
    return 0;
}
