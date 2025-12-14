#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamClient : public rclcpp::Node{
public:
    ParamClient(): Node("ParamClient"){
        RCLCPP_INFO(this->get_logger(),"参数客户端已创建!");
        client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"/turtlesim");
    }

    // 判断服务端是否开启
    bool is_server_exist(){
        while(!client_->wait_for_service(1s)){
            if (!rclcpp::ok()){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"客户端强制关闭!");
                return false;
            }
            RCLCPP_INFO(this->get_logger(),"等待服务连接中...");
        }
        return true;
    }

    // 更新参数
    void set_param(){
        int flag = 0;
        int red = client_->get_parameter<int>("background_r");
        rclcpp::Rate rate(30);
        while (rclcpp::ok()){
            if (red >= 255)  flag = 1;
            else if (red <= 0) flag = 0;
            
            if(flag)
                red -= 5;
            else
                red += 5;
            client_->set_parameters({rclcpp::Parameter("background_r",red)});
            rate.sleep();
        }
        
    }
private:
    rclcpp::SyncParametersClient::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);

    auto client = std::make_shared<ParamClient>();
    if(!client->is_server_exist()){
        RCLCPP_INFO(client->get_logger(),"客户端启动失败!");
    }
    else{
        RCLCPP_INFO(client->get_logger(),"客户端启动成功!");
        client->set_param();
    }

    rclcpp::shutdown();
    return 0;
}
