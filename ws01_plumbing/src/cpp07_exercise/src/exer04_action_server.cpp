#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/nav.hpp"

class ActionServer : public rclcpp::Node {
public:
    ActionServer() : Node("ActionServer"),x(0.0),y(0.0){
        RCLCPP_INFO(this->get_logger(),"action通信服务端已启动");
        sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",
            10,std::bind(&ActionServer::topic_callback,this,std::placeholders::_1));
        // 发布速度指令
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);

        action_server_ = rclcpp_action::create_server<base_interfaces_demo::action::Nav>(
            this,
            "nav",
            std::bind(&ActionServer::handle_goal,this,std::placeholders::_1,std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel,this,std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted,this,std::placeholders::_1)
        );
    }
    
private:
    // 订阅乌龟的实时位姿
    void topic_callback(turtlesim::msg::Pose::ConstSharedPtr topic){
        x = topic->x;
        y = topic->y;
    }
    // 接收请求值回调
    //GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    rclcpp_action::GoalResponse  handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const base_interfaces_demo::action::Nav::Goal> goal_handle){
        (void)uuid;
        // 目标点在范围内
        if (goal_handle->goal_x > 0 && goal_handle->goal_x < 11.08 && goal_handle->goal_y > 0 && goal_handle->goal_y < 11.08)
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        // 目标点超出范围内
        else{
            RCLCPP_WARN(this->get_logger(),"目标点超出取值范围!");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    // 接收取消请求回调
    // CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces_demo::action::Nav>> cancel_handle){
        (void)cancel_handle;
        RCLCPP_WARN(this->get_logger(),"取消任务");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 主逻辑回调
    // void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces_demo::action::Nav>> accepted_handle){
        (void)accepted_handle;
        std::thread(std::bind(&ActionServer::execute,this,accepted_handle)).detach();
    }

    // 子线程执行耗时操作
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<base_interfaces_demo::action::Nav>> accepted_handle){
        RCLCPP_INFO(this->get_logger(),"主逻辑开始执行!");
        rclcpp::Rate rate(1);
        auto result = std::make_shared<base_interfaces_demo::action::Nav::Result>();
        auto feedback = std::make_shared<base_interfaces_demo::action::Nav::Feedback>();
        geometry_msgs::msg::Twist goal;
        while (rclcpp::ok()){
            // 判断请求是否被取消
            if (accepted_handle->is_canceling()){
                accepted_handle->canceled(result);
                return;
            }
            // 解析目标点坐标与原生乌龟坐标

            // 计算相距距离
            float distance_x = accepted_handle->get_goal()->goal_x - this->x;
            float distance_y = accepted_handle->get_goal()->goal_y - this->y;
            float distance = std::sqrt(distance_x * distance_x + distance_y * distance_y);

            // 发布连续反馈
            //std::shared_ptr<base_interfaces_demo::action::Nav_Feedback> feedback_msg
            feedback->distance = distance;
            accepted_handle->publish_feedback(feedback);

            // 发布速度指令
            float index = 0.5;
            float linear_x = index * distance_x;
            float linear_y = index * distance_y;
            goal.linear.x = linear_x;
            goal.linear.y = linear_y;
            pub_->publish(goal);

            // 循环结束条件
            if (distance < 0.05){
                RCLCPP_INFO(this->get_logger(),"乌龟已经到达目标点附近!");
                break;
            }
            rate.sleep(); 
        }
        // 最终响应
        if (rclcpp::ok()){
            result->turtle_x = x;
            result->turtle_y = y;
            accepted_handle->succeed(result);
        }
    }
private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp_action::Server<base_interfaces_demo::action::Nav>::SharedPtr action_server_;
    float x;
    float y;
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ActionServer>());
    rclcpp::shutdown();
    return 0;
}
