#ifndef SENTRY_NAV_GOAL_HPP
#define SENTRY_NAV_GOAL_HPP

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

#endif