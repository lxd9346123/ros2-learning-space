/* 参数服务本质上是对服务通信的进一步封装 */
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 3.创建类对象
class MyParam_Client : public rclcpp::Node{
public:
    MyParam_Client() : Node("MyParam_Client"){
        RCLCPP_INFO(this->get_logger(),"参数客户端节点创建成功!");
        // 3.1创建参数客户端对象
        // 参数1：当前对象依赖的节点
        // 参数2：参数服务端节点名称
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"MyParam_Server");
    }
    // 3.2连接服务器
    bool connect_server(){
        while(!param_client_->wait_for_service(1s)){
            RCLCPP_INFO(this->get_logger(),"服务连接中!");
            if (!rclcpp::ok())
            return false;
        }
        return true;
    }
    // 3.3查询参数
    void get_param(void){
        RCLCPP_INFO(this->get_logger(),"----------查询参数----------");
        // get_parameter()
        std::string car_name = param_client_->get_parameter<std::string>("car_name");
        RCLCPP_INFO(this->get_logger(),"car_name = %s",car_name.c_str());
        
        // get_parameters()
        auto params = param_client_->get_parameters({"car_name","car_height","car_wheels"});
        for (auto &&param : params){
            RCLCPP_INFO(this->get_logger(),"%s = %s",param.get_name().c_str(),param.value_to_string().c_str());
        }
        
        // has_parameter()
        RCLCPP_INFO(this->get_logger(),"是否包含'car_name'? %d",param_client_->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(),"是否包含'test'? %d",param_client_->has_parameter("test"));
        
    } 
    //  3.4修改参数
    void update_param(void){
        RCLCPP_INFO(this->get_logger(),"----------修改参数----------");
        param_client_->set_parameters({
            rclcpp::Parameter("car_name","Ford"),
            rclcpp::Parameter("car_height",2.00),
            rclcpp::Parameter("car_wheels",4),
            // 这里新增一个原服务端不存在的参数，前提条件时服务端初始化时设置rclcpp::NodeOptions().allow_undeclared_parameters(true)
            rclcpp::Parameter("car_length",3.3)
        });
        RCLCPP_INFO(this->get_logger(),"是否包含'car_length'? %d",param_client_->has_parameter("car_length"));
    } 
private:
    rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char const *argv[])
{
    // 1.初始化节点
    rclcpp::init(argc,argv);

    auto node = std::make_shared<MyParam_Client>();
    
    if (!node->connect_server()){
        RCLCPP_WARN(node->get_logger(),"连接服务中断!");
        return 1;
    }

    node->get_param();
    node->update_param();
    node->get_param();
    
    // 4.资源释放
    rclcpp::shutdown();
    return 0;
}
