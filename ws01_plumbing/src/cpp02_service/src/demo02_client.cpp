#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;

class AddIntsClient : public rclcpp::Node{
public:
    // 构造函数
    AddIntsClient() : Node("AddIntsClient"){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"客户端节点创建成功!");  //输出日志
        client_ = this->create_client<AddInts>("add_ints");  //创建客户端
    }
    // 等待服务端开启
    bool Wait_For_Server(){
        while (!client_->wait_for_service(2s)){
            if (!rclcpp::ok()){
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"按下Ctrl+C,客户端退出");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"等待服务端...");
        }
        return true;
    }
    // 发送请求
    rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1,int num2){
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
        return client_->async_send_request(request);
    }
private:
    rclcpp::Client<AddInts>::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    // 1.判断传入参数个数是否正确
    if (argc != 3){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"请传入两个整型数据!");
        return 1;
    }
    // 2.初始化节点
    rclcpp::init(argc,argv);
    // 3.创建client对象
    auto client = std::make_shared<AddIntsClient>();
    // 4.等待服务端开启
    auto flag = client->Wait_For_Server();
    if (!flag){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"服务器连接失败,程序退出!");
        return 0;
    }
    // 5.发送请求
    auto result = client->send_request(atoi(argv[1]),atoi(argv[2]));
    // 响应成功
    if (rclcpp::spin_until_future_complete(client,result) == rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_INFO(client->get_logger(),"响应成功! Sum = %d",result.get()->sum);
    }
    // 响应失败
    else{
        RCLCPP_ERROR(client->get_logger(),"响应失败!");
    }
    rclcpp::shutdown();
    return 0;
}
