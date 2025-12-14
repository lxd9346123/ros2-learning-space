#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/distance.hpp"

using base_interfaces_demo::srv::Distance;
using namespace std::chrono_literals;

class exer_client : public rclcpp::Node{
public:
    exer_client() : Node("exer_client"){
        RCLCPP_INFO(this->get_logger(),"案例2服务通信客户端已创建!");
        client_ = this->create_client<Distance>("/calculate");
    }
    // 判断服务端是否开启
    bool is_server_exist(){
        while(!client_->wait_for_service(1s)){
            // 若中途推出则返回false
            if (!rclcpp::ok()){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"客户端被强制中断!");
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"等待服务端连接...");
        }
        return true;
    }
    // 发送请求数据
    rclcpp::Client<Distance>::FutureAndRequestId send_goals(const float& x,const float& y,const float& theta){
        auto request = std::make_shared<Distance::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        return client_->async_send_request(request);
    }
private:
    rclcpp::Client<Distance>::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    float goal_x;
    float goal_y;
    float goal_theta;
    // 判断传入参数个数
    if (argc != 5){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请传入参数信息!");
        return 1;
    }
    // 将参数传入
    else{
        goal_x = atof(argv[1]);
        goal_y = atof(argv[2]);
        goal_theta = atof(argv[3]);
    }
    // 节点初始化
    rclcpp::init(argc,argv);
    // 创建客户端对象
    auto client = std::make_shared<exer_client>();
    // 判断服务端是否链接上
    bool flag = client->is_server_exist();
    if (!flag)
        RCLCPP_INFO(client->get_logger(),"服务端连接失败!");
    else{
        // 发送请求数据
        auto future = client->send_goals(goal_x,goal_y,goal_theta);
        // 判断是否解算成功
        if (rclcpp::spin_until_future_complete(client,future) == rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_INFO(client->get_logger(),"两只乌龟的距离:%.2f米",future.get()->distance);
        }
        else
            RCLCPP_INFO(client->get_logger(),"解算失败!");
    }
    // 释放资源
    rclcpp::shutdown();
    return 0;
}
