#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/action/progress.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;

// 3.类创建
class ProgressActionServer : public rclcpp::Node{
public:
    // 3.1构造函数
    ProgressActionServer() : Node("ProgressActionServer"){
        RCLCPP_INFO(this->get_logger(),"action通信服务端节点已创建!");
        server_ = rclcpp_action::create_server<Progress>(
            this,
            "get_sum",
            std::bind(&ProgressActionServer::handle_goal,this,_1,_2),
            std::bind(&ProgressActionServer::handle_cancel,this,_1),
            std::bind(&ProgressActionServer::handle_accepted,this,_1)
        );
    }
    // 3.2处理提交的目标值(回调函数)
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Progress::Goal> goal_handle){
        if (goal_handle->num <= 1){
            RCLCPP_INFO(this->get_logger(),"提交的数字必须大于1!");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(),"提交的数字合法!");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // 3.3处理发送的取消请求(回调函数)
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> cancel_handle){
        RCLCPP_INFO(this->get_logger(),"接收到任务取消请求!");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    // 3.4生成连续反馈和最终响应(回调函数)
    // 新建子线程
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> accepted_handle){
        // 生成连续反馈
        int num = accepted_handle->get_goal()->num;
        int sum = 0;
        rclcpp::Rate rate(1.0);
        auto feedback = std::make_shared<Progress::Feedback>();
        auto result = std::make_shared<Progress::Result>();
        for (int i = 1; i <= num;i++){
            // 判断任务是否被取消
            if (accepted_handle->is_canceling()){
                accepted_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(),"任务被取消了!");
                return;
            }
            sum += i;
            double progress = i / (double)num;
            feedback->progress = progress;
            accepted_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(),"连续反馈中,进度:%.2f",feedback->progress);

            rate.sleep();
        }
        // 返回最终结果
        if (rclcpp::ok()){
            result->sum = sum;
            accepted_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(),"返回结果:%d",result->sum);
        }
    }
    // 回调函数
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> accepted_handle){
        // 新建子线程处理耗时的主逻辑实现
        std::thread(std::bind(&ProgressActionServer::execute,this,accepted_handle)).detach();
    }
private:
    rclcpp_action::Server<Progress>::SharedPtr server_;
};

int main(int argc, char const *argv[])
{
    // 1.节点初始化
    rclcpp::init(argc,argv);
    // 2.spin函数
    rclcpp::spin(std::make_shared<ProgressActionServer>());
    // 4.释放资源
    rclcpp::shutdown();
    return 0;
}
