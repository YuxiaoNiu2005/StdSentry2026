// #include "sentry_control/sentry_nav_goal.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_nav_decision_interface/msg/referee.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

typedef struct
{
    uint16_t current_hp;            // 当前血量
    uint8_t bullet_remaining_num;   // 剩余子弹数
    bool is_start_game;             // 是否开始比赛
} referee_t;

enum class sentry_state_navE
{
    WAIT_FOR_START = 0,     // 等待比赛开始
    MOVE_TO_STRATEGIC,      // 移动到战略点
    MOVE_TO_HIDE,           // 剩余发单量为0时，移动到躲藏点
    MOVE_TO_HEAL,           // 移动到恢复点
    WAIT_FOR_HEAL           // 等待回血，关闭小陀螺
};

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class SentryNavigator : public rclcpp::Node
{
private:
    bool goal_done;
    rclcpp::Subscription<sentry_nav_decision_interface::msg::Referee>::SharedPtr sub_referee;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_goal_client_ptr;
    sentry_state_navE sentry_state;
    referee_t referee_data;
    bool is_small_gyro;
    int retry_count;    // 导航失败尝试次数
    bool change_create_goal;
    geometry_msgs::msg::PoseStamped last_goal;  // 记录最后一个导航目标

    geometry_msgs::msg::PoseStamped recover_zone;
    geometry_msgs::msg::PoseStamped strategic_zone;
    geometry_msgs::msg::PoseStamped hide_zone;

    void goal_result_callback(const GoalHandleNav::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully.");
            retry_count = 0;  // 重置重试计数
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to reach goal.");
            if (retry_count < 2)
            {  
                // 失败时，最多再重试 2 次
                retry_count++;
                RCLCPP_WARN(this->get_logger(), "Retrying (%d/2)...", retry_count);
                send_goal(last_goal);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Navigation failed after 3 attempts.");
            }
        }
    }

    // 发送导航目标位置
    void send_goal(const geometry_msgs::msg::PoseStamped &goal)
    {
        if (!nav_goal_client_ptr->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal;
        last_goal = goal;  // 记录当前目标点
        retry_count = 0;   // 重置重试计数

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&SentryNavigator::goal_result_callback, this, std::placeholders::_1);

        RCLCPP_INFO(this->get_logger(), "Sending navigation goal: x=%.2f, y=%.2f", 
                    goal.pose.position.x, goal.pose.position.y);

        nav_goal_client_ptr->async_send_goal(goal_msg, send_goal_options);
    }

    void sub_referee_callback(const sentry_nav_decision_interface::msg::Referee::SharedPtr msg_ptr)
    {
        referee_data.current_hp = msg_ptr->current_hp;
        referee_data.bullet_remaining_num = msg_ptr->bullet_remaining_num;
        referee_data.is_start_game = msg_ptr->is_start_game;

        if (referee_data.current_hp < 395 && !change_create_goal)
        {
            change_create_goal = true;
        }

        if (change_create_goal)
        {
            strategic_zone = create_goal(4.8, 2.3, -0.8);
        }
        
        
        if (!referee_data.is_start_game)
        {
            sentry_state = sentry_state_navE::WAIT_FOR_START;  // 比赛未开始时，重置状态
            return;
        }

        switch (sentry_state) {
            case sentry_state_navE::WAIT_FOR_START:
                if (referee_data.is_start_game)
                {
                    sentry_state = sentry_state_navE::MOVE_TO_STRATEGIC;
                    send_goal(strategic_zone);
                }
                break;

            case sentry_state_navE::MOVE_TO_STRATEGIC:
                if (referee_data.current_hp < 200)
                {
                    sentry_state = sentry_state_navE::MOVE_TO_HEAL;
                    send_goal(recover_zone);
                }
                else if (referee_data.bullet_remaining_num <= 0)
                {
                    sentry_state = sentry_state_navE::MOVE_TO_HIDE;
                    send_goal(hide_zone);
                }
                break;

            case sentry_state_navE::MOVE_TO_HIDE:
                if (referee_data.current_hp < 200)
                {
                    sentry_state = sentry_state_navE::MOVE_TO_HEAL;
                    send_goal(recover_zone);
                }
                break;

            case sentry_state_navE::MOVE_TO_HEAL:
                is_small_gyro = false;  // 进入回血状态，关闭小陀螺，导航不准时，以便于机器人将哨兵撞回恢复区
                sentry_state = sentry_state_navE::WAIT_FOR_HEAL;
                break;

            case sentry_state_navE::WAIT_FOR_HEAL:
                if (referee_data.current_hp >= 380)
                {
                    is_small_gyro = true;  // 回血完成，开启小陀螺
                    if (referee_data.bullet_remaining_num > 0)
                    {
                        sentry_state = sentry_state_navE::MOVE_TO_STRATEGIC;
                        send_goal(strategic_zone);
                    }
                    else
                    {
                        sentry_state = sentry_state_navE::MOVE_TO_HIDE;
                        send_goal(hide_zone);
                    }
                }
                break;
        }
    }
    geometry_msgs::msg::PoseStamped create_goal(double x, double y, double yaw)
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.orientation.z = sin(yaw / 2.0);
        goal.pose.orientation.w = cos(yaw / 2.0);
        return goal;
    }
public:
    SentryNavigator(std::string name) : Node(name)
    {
        goal_done = false;
        this->nav_goal_client_ptr = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
        sub_referee = this->create_subscription<sentry_nav_decision_interface::msg::Referee>(
            "/referee",
            10, 
            std::bind(&SentryNavigator::sub_referee_callback,this, std::placeholders::_1)
        );
        sentry_state = sentry_state_navE::WAIT_FOR_START;
        is_small_gyro = true;
        retry_count = 0;
        change_create_goal = false;

        recover_zone = create_goal(-0.1, -0.2, -0.8);
        strategic_zone = create_goal(4.8, 2.3, -0.8);
        // hide_zone = create_goal(1.2, 1.2, -0.8);
        hide_zone = create_goal(4.8, 2.3, -0.8);
        RCLCPP_INFO(this->get_logger(), "sentry_making_decision!");
    }
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SentryNavigator>("sentry_nav_goal");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}