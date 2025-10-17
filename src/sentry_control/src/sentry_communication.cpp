#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sentry_control/sentry_communication.hpp"
#include "serial/serial.h"
#include "chrono"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "auto_aim_interfaces/msg/armor_choose.hpp"
#include "auto_aim_interfaces/msg/gimbal_cmd.hpp"
#include "auto_aim_interfaces/msg/serial_recive_data.hpp"
#include "sentry_nav_decision_interface/msg/referee.hpp"

/*
    �ýڵ���������λ��(ʹ��USB����ģ��usart)ͨѶ�����յ���λ��������δ��������ֱ�ӷ���ԭʼ���ݵ������У��쳣���
*/

std::shared_ptr<serial::Serial> ros_serial_ptr;

class USBCommunicationNode : public rclcpp::Node
{
    private:
        uint16_t reset_gimbal_cmd;
        uint16_t reset_cmd_vel;
        usb_transmit_u usb_transmit;
        usb_callback_u usb_callback_union;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::CallbackGroup::SharedPtr callbackgroup;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
        rclcpp::Subscription<auto_aim_interfaces::msg::ArmorChoose>::SharedPtr sub_armor_choose;
        rclcpp::Subscription<auto_aim_interfaces::msg::GimbalCmd>::SharedPtr sub_gimbal_cmd;
        rclcpp::Publisher<auto_aim_interfaces::msg::SerialReciveData>::SharedPtr pub_serial_recive_data;
        rclcpp::Publisher<sentry_nav_decision_interface::msg::Referee>::SharedPtr pub_referee;

        void sub_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg_ptr)
        {
            usb_callback_union.usb_callback.cmd_vel.linear.x = msg_ptr->linear.x;
            usb_callback_union.usb_callback.cmd_vel.linear.y = msg_ptr->linear.y;
            usb_callback_union.usb_callback.cmd_vel.linear.z = msg_ptr->linear.z;
            usb_callback_union.usb_callback.cmd_vel.angular.x = msg_ptr->angular.x;
            usb_callback_union.usb_callback.cmd_vel.angular.y = msg_ptr->angular.y;
            usb_callback_union.usb_callback.cmd_vel.angular.z = msg_ptr->angular.z;
            reset_cmd_vel = 0;
            // node_error_detect(this->get_node_names());
            usb_callback_union.usb_callback.error_type = 0;
        }
        void sub_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_ptr)
        {
            tf2::Quaternion quat(msg_ptr->orientation.x, msg_ptr->orientation.y, msg_ptr->orientation.z, msg_ptr->orientation.w);
            tf2::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);
            // RCLCPP_INFO(this->get_logger(), "yaw:%f", yaw);
            usb_callback_union.usb_callback.mid_360_yaw = (float)yaw;
        }
        void sub_armor_choose_callback(const auto_aim_interfaces::msg::ArmorChoose::SharedPtr msg_ptr)
        {
            // usb_callback_union.usb_callback.camera_feedback_data.base_gimbal = static_cast<u_int8_t>(msg_ptr->direc_flag);
        }
        void sub_gimbal_cmd_callback(const auto_aim_interfaces::msg::GimbalCmd::SharedPtr msg_ptr)
        {
            usb_callback_union.usb_callback.camera_feedback_data.letf_yaw = static_cast<float>(msg_ptr->yaw_1);
            usb_callback_union.usb_callback.camera_feedback_data.letf_pitch = static_cast<float>(msg_ptr->pitch_1);
            usb_callback_union.usb_callback.camera_feedback_data.right_yaw = static_cast<float>(msg_ptr->yaw_2);
            usb_callback_union.usb_callback.camera_feedback_data.right_pitch = static_cast<float>(msg_ptr->pitch_2);
            usb_callback_union.usb_callback.camera_feedback_data.fire_advice_left = static_cast<u_int8_t>(msg_ptr->fire_advice_left);
            usb_callback_union.usb_callback.camera_feedback_data.fire_advice_right = static_cast<u_int8_t>(msg_ptr->fire_advice_right);
            usb_callback_union.usb_callback.camera_feedback_data.is_tracking_left = static_cast<u_int8_t>(msg_ptr->is_tracking_left);
            usb_callback_union.usb_callback.camera_feedback_data.is_tracking_right = static_cast<u_int8_t>(msg_ptr->is_tracking_right);
            reset_gimbal_cmd = 0;
        }
        // /*
        //     error_type:
        //         1: ��λ�����յ�����(����λ��������)
        //         2: ������ͷ����(����λ��������)
        //         3: ������ͷ����(����λ��������)
        // */
        // void node_error_detect(const std::vector<std::string>& names) 
        // {
        //     bool is_run_VisionUp = false;
        //     bool is_run_VisionDown = false;
        //     for (const std::string & name : names)
        //     {
        //         if (name == "/VisionUp")
        //             is_run_VisionUp = true;
        //         if (name == "/VisionDown")
        //             is_run_VisionDown = true;
        //     }
        //     if (!is_run_VisionUp)
        //     {
        //         usb_callback_union.usb_callback.error_type = 2;
        //         return;
        //     }
        //     if (!is_run_VisionDown)
        //     {
        //         usb_callback_union.usb_callback.error_type = 3;
        //         return;
        //     }
        //     usb_callback_union.usb_callback.error_type = 0;
        // }
        void timer_callback()
        {
            if (reset_cmd_vel < 2000)
                reset_cmd_vel++;
            if (reset_cmd_vel >= 250)
            {
                usb_callback_union.usb_callback.cmd_vel.linear.x = 0.0f;
                usb_callback_union.usb_callback.cmd_vel.linear.y = 0.0f;
                usb_callback_union.usb_callback.cmd_vel.linear.z = 0.0f;
                usb_callback_union.usb_callback.cmd_vel.angular.x = 0.0f;
                usb_callback_union.usb_callback.cmd_vel.angular.y = 0.0f;
                usb_callback_union.usb_callback.cmd_vel.angular.z = 0.0f;
            }

            if (reset_gimbal_cmd < 2000)
                reset_gimbal_cmd++;
            if (reset_gimbal_cmd >= 250)
            {
                usb_callback_union.usb_callback.camera_feedback_data.letf_yaw = 0.0f;
                usb_callback_union.usb_callback.camera_feedback_data.letf_pitch = 0.0f;
                usb_callback_union.usb_callback.camera_feedback_data.right_yaw = 0.0f;
                usb_callback_union.usb_callback.camera_feedback_data.right_pitch = 0.0f;
                usb_callback_union.usb_callback.camera_feedback_data.fire_advice_left = 0;
                usb_callback_union.usb_callback.camera_feedback_data.fire_advice_right = 0;
                usb_callback_union.usb_callback.camera_feedback_data.is_tracking_left = 0;
                usb_callback_union.usb_callback.camera_feedback_data.is_tracking_right = 0;
            }
            
            
            // for (std::string & name : this->get_node_names())
            // {
            //     RCLCPP_INFO(this->get_logger(), "name:%s", name.c_str());
            //     RCLCPP_INFO(this->get_logger(), "-------------------------------");
            // }
            ros_serial_ptr -> write(usb_callback_union.data, sizeof(usb_callback_t));
            if(ros_serial_ptr->available())
            {
                auto_aim_interfaces::msg::SerialReciveData vision_msg;
                sentry_nav_decision_interface::msg::Referee referee_msg;
                ros_serial_ptr -> read(usb_transmit.data, sizeof(usb_transmit_t));
                // RCLCPP_INFO(this->get_logger(), "%f", this->float_union.float_data);
                // RCLCPP_INFO(this->get_logger(), "%d", sizeof(usb_transmit_t));
                vision_msg.header.stamp = this->get_clock()->now();
                vision_msg.header.frame_id = "communication";
                vision_msg.reset_tracker = false;
                vision_msg.detect_color = usb_transmit.usb_tx.camera_tx_data.detect_color;
                vision_msg.speed = usb_transmit.usb_tx.camera_tx_data.projectile_speed;;
                vision_msg.yaw = usb_transmit.usb_tx.camera_tx_data.parent_yaw;
                vision_msg.roll1 = 0.0f;
                vision_msg.pitch1 = usb_transmit.usb_tx.camera_tx_data.letf_pitch;
                vision_msg.yaw1 = usb_transmit.usb_tx.camera_tx_data.letf_yaw;
                vision_msg.roll2 = 0.0f;
                vision_msg.pitch2 = usb_transmit.usb_tx.camera_tx_data.right_pitch;
                vision_msg.yaw2 = usb_transmit.usb_tx.camera_tx_data.right_yaw;
                pub_serial_recive_data->publish(vision_msg);
                referee_msg.current_hp = usb_transmit.usb_tx.referee_tx_data.current_HP;
                referee_msg.bullet_remaining_num = usb_transmit.usb_tx.referee_tx_data.bullet_remaining_num;
                referee_msg.is_start_game = static_cast<bool>(usb_transmit.usb_tx.referee_tx_data.is_start_game);
                pub_referee->publish(referee_msg);
            }
        }
    public:
        USBCommunicationNode(std::string name) : Node(name)
        {
            callbackgroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            auto sub_options = rclcpp::SubscriptionOptions();
            sub_options.callback_group = callbackgroup;
            sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",
                                                                                10, 
                                                                                std::bind(&USBCommunicationNode::sub_cmd_vel_callback,this, std::placeholders::_1), 
                                                                                sub_options);
            sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("imu/data",
                                                                        10, 
                                                                        std::bind(&USBCommunicationNode::sub_imu_callback,this, std::placeholders::_1), 
                                                                        sub_options);
            sub_armor_choose = this->create_subscription<auto_aim_interfaces::msg::ArmorChoose>("/detector/armor_choose",
                                                                                                10, 
                                                                                                std::bind(&USBCommunicationNode::sub_armor_choose_callback,this, std::placeholders::_1), 
                                                                                                sub_options);
            sub_gimbal_cmd = this->create_subscription<auto_aim_interfaces::msg::GimbalCmd>("/GimbalCmd",
                                                                                            10, 
                                                                                            std::bind(&USBCommunicationNode::sub_gimbal_cmd_callback,this, std::placeholders::_1), 
                                                                                            sub_options);
            pub_serial_recive_data = this->create_publisher<auto_aim_interfaces::msg::SerialReciveData>("/SerialReciveData", 100);
            pub_referee = this->create_publisher<sentry_nav_decision_interface::msg::Referee>("/referee", 10);
            timer = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&USBCommunicationNode::timer_callback, this), callbackgroup);
            usb_callback_union.usb_callback.error_type = 0;
            reset_gimbal_cmd = 0;
            reset_cmd_vel = 0;
        }
};

int main(int argc, char * argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<USBCommunicationNode>("sentry_communication");
    // ros_serial_ptr = std::make_shared<serial::Serial>(USB_PORT, 115200, serial::Timeout::simpleTimeout(100));
    try
    {
        ros_serial_ptr = std::make_shared<serial::Serial>(USB_PORT, 115200, serial::Timeout::simpleTimeout(100));
        // ros_serial_ptr -> open();
    }
    catch(serial::IOException &e)
    {
        RCLCPP_ERROR(node_ptr->get_logger(), "unable to open:%s", e.what());
        return -1;
    }

    if (ros_serial_ptr -> isOpen())
    {
        RCLCPP_INFO(node_ptr->get_logger(), "SUCCEED OPEN %s", USB_PORT);
    }
    else
    {
        return -1;
    }
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_ptr);
    executor.spin();
    rclcpp::shutdown();
    ros_serial_ptr->close();
    
    return 0;
}
