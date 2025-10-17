#ifndef SENTRY_NAV_GOAL_HPP
#define SENTRY_NAV_GOAL_HPP

typedef struct
{
    uint16_t current_hp;            // ��ǰѪ��
    uint8_t bullet_remaining_num;   // ʣ���ӵ���
    bool is_start_game;             // �Ƿ�ʼ����
} referee_t;

enum class sentry_state_navE
{
    WAIT_FOR_START = 0,     // �ȴ�������ʼ
    MOVE_TO_STRATEGIC,      // �ƶ���ս�Ե�
    MOVE_TO_HIDE,           // ʣ�෢����Ϊ0ʱ���ƶ�����ص�
    MOVE_TO_HEAL,           // �ƶ����ָ���
    WAIT_FOR_HEAL           // �ȴ���Ѫ���ر�С����
};

#endif