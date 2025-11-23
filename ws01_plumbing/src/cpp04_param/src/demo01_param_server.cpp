/* 参数服务本质上是对服务通信的进一步封装 */
#include "rclcpp/rclcpp.hpp"

// 3.创建类对象
class MyParam_Server : public rclcpp::Node{
public:
    // 需要删除时要在构造函数新增传入参数值
    MyParam_Server() : Node("MyParam_Server",rclcpp::NodeOptions().allow_undeclared_parameters(true)){
        RCLCPP_INFO(this->get_logger(),"参数服务端节点创建成功!");
    }

    // 3.1增加参数
    void declare_param(void){
        RCLCPP_INFO(this->get_logger(),"-------------------增-------------------");
        this->declare_parameter("car_name","BWM");
        this->declare_parameter("car_height",1.68);
        this->declare_parameter("car_wheels",5);

        this->set_parameter(rclcpp::Parameter("car_width",1.55));
    }
    // 3.2查找参数
    void get_param(void){
        RCLCPP_INFO(this->get_logger(),"-------------------查-------------------");
        auto car = this->get_parameter("car_name");
        RCLCPP_INFO(this->get_logger(),"key = %s, value = %s",car.get_name().c_str(),car.value_to_string().c_str());
        
        auto params = this->get_parameters({"car_name","car_height","car_wheels"});
        for (auto &&param : params){
            RCLCPP_INFO(this->get_logger(),"%s = %s",param.get_name().c_str(),param.value_to_string().c_str());
        }
        RCLCPP_INFO(this->get_logger(),"是否包含'car_name'?  %d",this->has_parameter("car_name"));
        RCLCPP_INFO(this->get_logger(),"是否包含'width'?  %d",this->has_parameter("width"));
    }
    // 3.3修改参数
    void update_param(void){
        RCLCPP_INFO(this->get_logger(),"-------------------改-------------------");
        this->set_parameter(rclcpp::Parameter("car_height",1.75));
        auto car = this->get_parameter("car_height");
        RCLCPP_INFO(this->get_logger(),"%s = %s",car.get_name().c_str(),car.value_to_string().c_str());
    }
    // 3.4删除参数
    void delete_param(void){
        RCLCPP_INFO(this->get_logger(),"-------------------删-------------------");
        // this->undeclare_parameter("car_name");
        /* 
            删除不能操作declare_parameter函数创建的参数，
            只能删除set_parameter函数创建的参数
        */
        this->undeclare_parameter("car_width");
        RCLCPP_INFO(this->get_logger(),"删除后还包含car_width吗?  %d",this->has_parameter("car_width"));
    }
};

int main(int argc, char const *argv[])
{
    // 1.初始化节点
    rclcpp::init(argc,argv);
    // 2.spin函数
    auto node = std::make_shared<MyParam_Server>();

    node->declare_param();
    node->get_param();
    node->update_param();
    node->delete_param();

    rclcpp::spin(node);
    // 4.资源释放
    rclcpp::shutdown();
    return 0;
}
