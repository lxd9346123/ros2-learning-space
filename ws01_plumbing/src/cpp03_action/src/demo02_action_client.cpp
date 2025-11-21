#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"

using namespace std::chrono_literals;
using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;


class ProgressActionClient : public rclcpp::Node{
public:
  // 3.1构造函数
  ProgressActionClient() : Node("ProgressActionClient"){
    RCLCPP_INFO(this->get_logger(),"action通信客户端节点已创建!");
  }
  // 3.2客户端发送函数
  void Send_Goal(int num){
    
    client_ = rclcpp_action::create_client<Progress>(this,"get_sum");
    // 等待服务器5s
    if (!client_->wait_for_action_server(5s)){
      RCLCPP_INFO(this->get_logger(),"连接服务器失败!");
    }

    auto goal = Progress::Goal();
    goal.num = num;
    rclcpp_action::Client<Progress>::SendGoalOptions options;
    // 绑定回调函数
    options.goal_response_callback = std::bind(&ProgressActionClient::goal_response_callback,this,_1);
    options.feedback_callback = std::bind(&ProgressActionClient::feedback_callback,this,_1,_2);
    options.result_callback = std::bind(&ProgressActionClient::result_callback,this,_1);
    // 发送数据
    client_->async_send_goal(goal,options);
  }
  // 3.3发送中断回调函数
  void goal_response_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle){
    if (!goal_handle){
      RCLCPP_INFO(this->get_logger(),"目标请求被服务端拒绝!");
    }
    else{
      goal_handle_ = goal_handle;
      RCLCPP_INFO(this->get_logger(),"目标处理中!");
    }
  }
  // 3.4连续反馈函数
  void feedback_callback(rclcpp_action::ClientGoalHandle<Progress>::SharedPtr feedback_handle,const std::shared_ptr<const Progress::Feedback> feedback){
      if (!rclcpp::ok()){
        if (!feedback_handle)
          client_->async_cancel_goal(feedback_handle);
      }
      auto progress = feedback->progress;
      int pro = int(progress * 100);
      RCLCPP_INFO(this->get_logger(),"当前处理进度:%d%%",pro);
  }
  // 3.5结果返回函数
  void result_callback(const rclcpp_action::ClientGoalHandle<Progress>::WrappedResult &result){
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      RCLCPP_INFO(this->get_logger(),"最终响应结果:%d",result.result->sum);
    else if (result.code == rclcpp_action::ResultCode::ABORTED)
      RCLCPP_INFO(this->get_logger(),"被中断!");
    else if (result.code == rclcpp_action::ResultCode::CANCELED)
      RCLCPP_INFO(this->get_logger(),"被取消!");
    else
      RCLCPP_INFO(this->get_logger(),"未知异常!");
  }
  // 客户端被终止时向服务端发送终止指令
  void Cancel_Goal(void){
    if(this->goal_handle_){
        this->client_->async_cancel_goal(this->goal_handle_);
    }
  }
private:
  rclcpp_action::Client<Progress>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<Progress>::SharedPtr goal_handle_;

};

int main(int argc, char const *argv[])
{
  // 1.节点初始化
  rclcpp::init(argc,argv);
  // 判断传入参数个数
  if (argc != 2){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"传入参数个数错误!");
    return 1;
  }
  auto action_client = std::make_shared<ProgressActionClient>();
  // 向服务端发送数据
  action_client->Send_Goal(atoi(argv[1]));
  // 2.spin函数
  rclcpp::spin(action_client);

  // 向服务端发送终止指令
  action_client->Cancel_Goal();
  
  // 4.释放资源
  rclcpp::shutdown();
  return 0;
}
