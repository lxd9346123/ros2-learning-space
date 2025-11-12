#include "rclcpp/rclcpp.hpp"


/*   方式1（不被推荐）
int main(int argc,char const *argv[]){
  // 1.初始化
  rclcpp::init(argc,argv);
  // 2.创建节点
  auto node = rclcpp::Node::make_shared("hello_vscode_node_cpp");
  // 3.日志输出
  RCLCPP_INFO(node->get_logger(),"Hello VSCode C++ World!");
  // 4.释放资源
  rclcpp::shutdown();
  return 0;
} */

/* 方式2（推荐） */
class Mynode : public rclcpp::Node{
public:
          Mynode():Node("hello_node_cpp"){
              RCLCPP_INFO(this->get_logger(),"hello world!(继承的方式)");
          }
};


int main(int argc, char const *argv[])
{
  // 初始化
  rclcpp::init(argc,argv);
  // 实例化自定义类
  auto node = std::make_shared<Mynode>();

  rclcpp::shutdown();
  return 0;
}
