#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"

using base_interfaces_demo::action::Nav;
using namespace std::chrono_literals;

class ActionClient : public rclcpp::Node {
public:
    ActionClient() : Node("ActionClient"){
        RCLCPP_INFO(this->get_logger(),"action通信客户端已启动");
        client_ = rclcpp_action::create_client<Nav>(this,"nav");
    }

    void send_goal(float x,float y,float theta){
        Nav::Goal goal;
        rclcpp_action::Client<Nav>::SendGoalOptions options;
        // 1.连接服务端
        if (!client_->wait_for_action_server(10s)){
            RCLCPP_WARN(this->get_logger(),"服务端连接超时!");
            return;
        }
        // 2.组织数据并且发布
        //const base_interfaces_demo::action::Nav::Goal &goal, 
        //const rclcpp_action::Client<base_interfaces_demo::action::Nav>::SendGoalOptions &options
        goal.goal_x = x;
        goal.goal_y = y;
        goal.goal_theta = theta;

        options.goal_response_callback = std::bind(&ActionClient::goal_response_callback,this,std::placeholders::_1);
        options.feedback_callback = std::bind(&ActionClient::feedback_callback,this,std::placeholders::_1,std::placeholders::_2);
        options.result_callback = std::bind(&ActionClient::result_callback,this,std::placeholders::_1);

        client_->async_send_goal(goal,options);
    }
private:
    //void (typename GoalHandle::SharedPtr
    void goal_response_callback(rclcpp_action::ClientGoalHandle<Nav>::SharedPtr response_handle){
        if (!response_handle)
            RCLCPP_INFO(this->get_logger(),"请求目标非法");
        else
            RCLCPP_INFO(this->get_logger(),"请求数据被接收");
    }

    //std::function<void (
    //    typename ClientGoalHandle<ActionT>::SharedPtr,
    //    const std::shared_ptr<const Feedback>)>;
    void feedback_callback(rclcpp_action::ClientGoalHandle<Nav>::SharedPtr goal_handle,const std::shared_ptr<const Nav::Feedback> feedback){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"剩余的距离:%.2f",feedback->distance);
    }

    //void (const WrappedResult & result)
    void result_callback(const rclcpp_action::ClientGoalHandle<Nav>::WrappedResult& result){
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_INFO(this->get_logger(),"成功响应");
            RCLCPP_INFO(this->get_logger(),"乌龟最终位姿信息:(%.2f,%.2f,%.2f)",
            result.result->turtle_x,
            result.result->turtle_y,
            result.result->turtle_theta);
        }
        else
            RCLCPP_INFO(this->get_logger(),"响应失败");
    }

private:
    rclcpp_action::Client<Nav>::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    if (argc != 5){
        RCLCPP_ERROR(rclcpp::get_logger("argc"),"请传入合法的目标点数据!");
        return 1;
    }
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ActionClient>();
    node->send_goal(atof(argv[1]),atof(argv[2]),atof(argv[3]));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}