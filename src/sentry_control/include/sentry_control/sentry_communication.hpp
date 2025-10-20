#ifndef SENTRY_COMMUNICATION_HPP
#define SENTRY_COMMUNICATION_HPP

#define USB_PORT "/dev/ttyACM0"

#pragma pack(push, 1)

typedef struct
{
    float x;
    float y;
    float z;
} Vector3_t;

typedef struct
{
    Vector3_t linear;
    Vector3_t angular;
} cmd_vel_buffer_t;

typedef struct
{
    float letf_yaw;
    float letf_pitch;
    float right_yaw;
    float right_pitch;
    u_int8_t fire_advice_left;
    u_int8_t fire_advice_right;
    u_int8_t is_tracking_left;        // 捕捉到装甲板为true， 未捕捉到装甲板为false
    u_int8_t is_tracking_right;        // 捕捉到装甲板为true， 未捕捉到装甲板为false
} camera_feedback_t; // 自瞄程序反馈的数据

/*
    error_type:
        1: 下位机接收到数据(由下位机检测产生)
        2: 上摄像头离线(由上位机检测产生)
        3: 下摄像头离线(由上位机检测产生)
*/

typedef struct
{
    cmd_vel_buffer_t cmd_vel;
    camera_feedback_t camera_feedback_data;
    float mid_360_yaw;
    uint8_t error_type;         // 错误类型
} usb_callback_t;

typedef union
{
    usb_callback_t usb_callback;
    uint8_t data[sizeof(usb_callback_t)];
} usb_callback_u;

/*
    flag_byte
    bit0 : 遥控器模式选择，1为ROS模式(遥控器右边开关)
    bit1 : 0对应开关位置在上为蓝方，1对应开关位置在下为红方(遥控器左边开关)
*/
typedef struct
{
    uint8_t detect_color;   // 0:红色 1:蓝色
    float projectile_speed;  // 弹速
    float parent_yaw;
    float letf_yaw;
    float letf_pitch;
    float right_yaw;
    float right_pitch;
} camera_tx_t; // 向上位机自瞄程序发送的数据

typedef struct
{
    uint16_t current_HP;            // 当前血量
    uint8_t bullet_remaining_num;   // 剩余子弹数
    uint8_t is_start_game;             // 是否开始比赛
} referee_tx_t;

typedef struct
{
    camera_tx_t camera_tx_data;
    referee_tx_t referee_tx_data;
} usb_transmit_t;

typedef union
{
    usb_transmit_t usb_tx;
    uint8_t data[sizeof(usb_transmit_t)];
} usb_transmit_u;

#pragma pack(pop)

typedef union
{
    float float_data;
    uint8_t data[4];
} float_u;

#endif