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
    u_int8_t is_tracking_left;        // ��׽��װ�װ�Ϊtrue�� δ��׽��װ�װ�Ϊfalse
    u_int8_t is_tracking_right;        // ��׽��װ�װ�Ϊtrue�� δ��׽��װ�װ�Ϊfalse
} camera_feedback_t; // ���������������

/*
    error_type:
        1: ��λ�����յ�����(����λ��������)
        2: ������ͷ����(����λ��������)
        3: ������ͷ����(����λ��������)
*/

typedef struct
{
    cmd_vel_buffer_t cmd_vel;
    camera_feedback_t camera_feedback_data;
    float mid_360_yaw;
    uint8_t error_type;         // ��������
} usb_callback_t;

typedef union
{
    usb_callback_t usb_callback;
    uint8_t data[sizeof(usb_callback_t)];
} usb_callback_u;

/*
    flag_byte
    bit0 : ң����ģʽѡ��1ΪROSģʽ(ң�����ұ߿���)
    bit1 : 0��Ӧ����λ������Ϊ������1��Ӧ����λ������Ϊ�췽(ң������߿���)
*/
typedef struct
{
    uint8_t detect_color;   // 0:��ɫ 1:��ɫ
    float projectile_speed;  // ����
    float parent_yaw;
    float letf_yaw;
    float letf_pitch;
    float right_yaw;
    float right_pitch;
} camera_tx_t; // ����λ����������͵�����

typedef struct
{
    uint16_t current_HP;            // ��ǰѪ��
    uint8_t bullet_remaining_num;   // ʣ���ӵ���
    uint8_t is_start_game;             // �Ƿ�ʼ����
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