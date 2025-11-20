#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"

using base_interfaces_demo::srv::AddInts;
using std::placeholders::_1;
using std::placeholders::_2;

// 3.服务端类创建
class AddIntsServer : public rclcpp::Node{
public:
  // 构造函数
  AddIntsServer() : Node("AddIntsServer"){
    RCLCPP_INFO(this->get_logger(),"服务端节点创建成功!");
    server_ = this->create_service<AddInts>("add_ints",std::bind(&AddIntsServer::add,this,_1,_2));
  }
private:
  // 回调函数
  void add(const AddInts::Request::SharedPtr req,const AddInts::Response::SharedPtr res){
    res->sum = req->num1 + req->num2;
    RCLCPP_INFO(this->get_logger(),"%d + %d = %d",req->num1,req->num2,res->sum);
  }
  rclcpp::Service<AddInts>::SharedPtr server_;
};

int main(int argc, char const *argv[])
{
  // 1.初始化节点
  rclcpp::init(argc,argv);
  // 2.spin函数
  rclcpp::spin(std::make_shared<AddIntsServer>());
  // 4.释放资源
  rclcpp::shutdown();
  return 0;
}

