# Sentry Control Package 文档

## 📋 功能包概述

`sentry_control` 是 Sentry 哨兵机器人的控制功能包,主要负责 **上位机(ROS2 系统)与下位机(硬件控制板)之间的通信**。

### 核心作用
- 🔗 通过 USB 串口建立通信桥梁
- 📤 向下位机发送速度命令、云台控制等指令
- 📥 接收下位机的传感器反馈、裁判系统数据
- 🛡️ 实现命令超时保护机制,防止机器人失控

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                   ROS2 上位机系统                            │
│  ┌────────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │   导航系统      │  │   视觉系统    │  │   决策系统   │    │
│  │ (Navigation)   │  │   (Vision)   │  │ (Decision)   │    │
│  └────────┬────────┘  └──────┬───────┘  └──────┬───────┘    │
│           │                  │                  │             │
│           └──────────────────┼──────────────────┘             │
│                              │                               │
│                    发布各类 ROS2 话题                        │
│                    (/cmd_vel, /GimbalCmd,                  │
│                     imu/data, etc.)                         │
│                              │                               │
│                              ▼                               │
│     ┌──────────────────────────────────────────────┐        │
│     │   USBCommunicationNode (本功能包)             │        │
│     │  - 序列化 ROS2 消息为二进制数据                │        │
│     │  - 通过串口发送给下位机                      │        │
│     │  - 接收下位机反馈并解析                      │        │
│     │  - 发布解析后的 ROS2 消息                    │        │
│     └────────────────────┬─────────────────────────┘        │
└─────────────────────────┼──────────────────────────────────┘
                          │
                    USB 串口 (115200)
                    双向数据通信
                          │
      ┌───────────────────┴───────────────────┐
      │                                       │
      ▼                                       ▼
┌──────────────────────┐          ┌──────────────────────┐
│   下位机控制板        │          │   下位机反馈数据      │
│  (STM32等)           │          │  (传感器、裁判)     │
│                      │          │                      │
│  - 速度控制          │◄────────►│  - 云台反馈角度     │
│  - 云台控制          │          │  - 当前血量         │
│  - 发射控制          │          │  - 弹量剩余         │
└──────────┬───────────┘          └──────────┬──────────┘
           │                                  │
           ▼                                  ▼
    ┌──────────────┐               ┌──────────────┐
    │   电机驱动    │               │   传感器反馈  │
    │   执行器      │               │   硬件状态    │
    └──────────────┘               └──────────────┘
```

---

## 📦 核心数据结构

### 接收数据 (ROS2 → 下位机)

通过联合体 `usb_callback_u` 封装,包含以下数据:

#### 速度命令 (`cmd_vel`)
```cpp
struct cmd_vel_t {
    float linear_x;   // 前后速度 (m/s)
    float linear_y;   // 左右速度 (m/s)
    float linear_z;   // 竖直速度 (m/s)
    float angular_x;  // 滚转角速度 (rad/s)
    float angular_y;  // 俯仰角速度 (rad/s)
    float angular_z;  // 偏航角速度 (rad/s)
}
```

#### IMU 数据 (四元数 → 欧拉角)
```cpp
float mid_360_yaw;  // MID360 激光雷达的偏航角 (rad)
```

#### 云台和视觉反馈 (`camera_feedback_data`)
```cpp
struct camera_feedback_t {
    float letf_yaw;                // 左云台偏航角
    float letf_pitch;              // 左云台俯仰角
    float right_yaw;               // 右云台偏航角
    float right_pitch;             // 右云台俯仰角
    uint8_t fire_advice_left;      // 左侧开火建议 (0/1)
    uint8_t fire_advice_right;     // 右侧开火建议 (0/1)
    uint8_t is_tracking_left;      // 左侧追踪状态
    uint8_t is_tracking_right;     // 右侧追踪状态
}
```

#### 错误类型
```cpp
uint8_t error_type;  
// 0: 正常
// 1: 基层通信接收错误
// 2: 上层视觉节点不运行
// 3: 下层视觉节点不运行
```

---

### 发送数据 (下位机 → ROS2)

通过联合体 `usb_transmit_u` 封装,包含以下数据:

#### 相机发送数据 (`camera_tx_data`)
```cpp
struct camera_tx_t {
    uint8_t detect_color;          // 检测颜色 (0=蓝, 1=红)
    float projectile_speed;        // 弹丸初速 (m/s)
    float parent_yaw;              // 父云台偏航角
    float letf_yaw;                // 左云台偏航角反馈
    float letf_pitch;              // 左云台俯仰角反馈
    float right_yaw;               // 右云台偏航角反馈
    float right_pitch;             // 右云台俯仰角反馈
}
```

#### 裁判系统数据 (`referee_tx_data`)
```cpp
struct referee_tx_t {
    uint16_t current_HP;           // 当前血量 (HP)
    uint16_t bullet_remaining_num; // 剩余弹量
    uint8_t is_start_game;         // 游戏是否开始 (0/1)
}
```

---

## 📡 ROS2 话题接口

### 订阅的话题 (输入)

| 话题名称 | 消息类型 | 发布者 | 功能描述 |
|---------|---------|--------|---------|
| `/cmd_vel` | `geometry_msgs/Twist` | 导航系统 | 速度命令 (线速度 + 角速度) |
| `imu/data` | `sensor_msgs/Imu` | IMU 驱动 | IMU 传感器数据 (四元数姿态) |
| `/detector/armor_choose` | `ArmorChoose` | 视觉系统 | 装甲板选择 |
| `/GimbalCmd` | `GimbalCmd` | 视觉系统 | 云台控制命令 |

### 发布的话题 (输出)

| 话题名称 | 消息类型 | 订阅者 | 功能描述 |
|---------|---------|--------|---------|
| `/SerialReciveData` | `SerialReciveData` | 视觉/决策系统 | 来自下位机的视觉反馈数据 |
| `/referee` | `Referee` | 导航/决策系统 | 来自下位机的裁判系统信息 |

---

## 🔄 数据流处理

### 📥 输入流程 (ROS2 → 下位机)

```
1. 订阅 ROS2 话题
   ↓
2. 在回调函数中解析消息内容
   ↓
3. 填充到 usb_callback_union 结构体中
   ↓
4. 重置超时计数器 (看门狗机制)
   ↓
5. 等待定时器触发
```

**具体例子:**

```cpp
void sub_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg_ptr)
{
    // 解析速度消息
    usb_callback_union.usb_callback.cmd_vel.linear.x = msg_ptr->linear.x;
    usb_callback_union.usb_callback.cmd_vel.linear.y = msg_ptr->linear.y;
    usb_callback_union.usb_callback.cmd_vel.linear.z = msg_ptr->linear.z;
    usb_callback_union.usb_callback.cmd_vel.angular.x = msg_ptr->angular.x;
    usb_callback_union.usb_callback.cmd_vel.angular.y = msg_ptr->angular.y;
    usb_callback_union.usb_callback.cmd_vel.angular.z = msg_ptr->angular.z;
    
    // 重置超时计数器
    reset_cmd_vel = 0;
    
    // 清除错误标志
    usb_callback_union.usb_callback.error_type = 0;
}
```

### 📤 输出流程 (下位机 → ROS2)

```
1. 定时器触发 (每 2ms)
   ↓
2. 通过串口发送数据到下位机
   ↓
3. 检查串口是否有数据返回
   ↓
4. 读取下位机的二进制数据
   ↓
5. 解析数据填充到 usb_transmit 结构体
   ↓
6. 转换为 ROS2 消息格式
   ↓
7. 发布到对应的 ROS2 话题
```

**具体例子:**

```cpp
if(ros_serial_ptr->available())
{
    // 读取下位机数据
    ros_serial_ptr->read(usb_transmit.data, sizeof(usb_transmit_t));
    
    // 创建视觉反馈消息
    auto_aim_interfaces::msg::SerialReciveData vision_msg;
    vision_msg.header.stamp = this->get_clock()->now();
    vision_msg.header.frame_id = "communication";
    vision_msg.detect_color = usb_transmit.usb_tx.camera_tx_data.detect_color;
    vision_msg.speed = usb_transmit.usb_tx.camera_tx_data.projectile_speed;
    vision_msg.yaw = usb_transmit.usb_tx.camera_tx_data.parent_yaw;
    vision_msg.pitch1 = usb_transmit.usb_tx.camera_tx_data.letf_pitch;
    vision_msg.yaw1 = usb_transmit.usb_tx.camera_tx_data.letf_yaw;
    // ... 更多字段
    
    // 发布消息
    pub_serial_recive_data->publish(vision_msg);
    
    // 创建裁判系统消息
    sentry_nav_decision_interface::msg::Referee referee_msg;
    referee_msg.current_hp = usb_transmit.usb_tx.referee_tx_data.current_HP;
    referee_msg.bullet_remaining_num = usb_transmit.usb_tx.referee_tx_data.bullet_remaining_num;
    referee_msg.is_start_game = usb_transmit.usb_tx.referee_tx_data.is_start_game;
    
    // 发布消息
    pub_referee->publish(referee_msg);
}
```

---

## 🛡️ 安全机制

### 看门狗 (Watchdog) - 命令超时保护

**目的:** 防止导航系统或视觉系统宕机导致机器人失控

**机制:**

```cpp
void timer_callback()  // 每 2ms 执行一次
{
    // 跟踪 cmd_vel 接收时间
    if (reset_cmd_vel < 2000)
        reset_cmd_vel++;
    
    // 超过 500ms (250 * 2ms) 没收到新命令
    if (reset_cmd_vel >= 250)
    {
        // 清零所有速度命令
        usb_callback_union.usb_callback.cmd_vel.linear.x = 0.0f;
        usb_callback_union.usb_callback.cmd_vel.linear.y = 0.0f;
        usb_callback_union.usb_callback.cmd_vel.linear.z = 0.0f;
        usb_callback_union.usb_callback.cmd_vel.angular.x = 0.0f;
        usb_callback_union.usb_callback.cmd_vel.angular.y = 0.0f;
        usb_callback_union.usb_callback.cmd_vel.angular.z = 0.0f;
    }
    
    // 同理处理云台命令超时
    if (reset_gimbal_cmd < 2000)
        reset_gimbal_cmd++;
    
    if (reset_gimbal_cmd >= 250)
    {
        // 清零所有云台命令
        usb_callback_union.usb_callback.camera_feedback_data.letf_yaw = 0.0f;
        usb_callback_union.usb_callback.camera_feedback_data.letf_pitch = 0.0f;
        // ... 更多字段
    }
}
```

**超时参数:**
- 超时时间: **500ms** (250 * 2ms)
- 意义: 如果 500ms 内没有新的命令更新,立即停止机器人

---

## 🧮 核心计算 - IMU 四元数转欧拉角

```cpp
void sub_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_ptr)
{
    // 步骤 1: 从 ROS2 消息提取四元数
    tf2::Quaternion quat(
        msg_ptr->orientation.x,
        msg_ptr->orientation.y,
        msg_ptr->orientation.z,
        msg_ptr->orientation.w
    );
    
    // 步骤 2: 转换为旋转矩阵
    tf2::Matrix3x3 mat(quat);
    
    // 步骤 3: 从旋转矩阵提取欧拉角 (Roll, Pitch, Yaw)
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    
    // 步骤 4: 只发送偏航角给下位机
    usb_callback_union.usb_callback.mid_360_yaw = (float)yaw;
}
```

**为什么需要转换?**
- ROS2 IMU 使用四元数表示姿态 (旋转角度表示,无冗余)
- 下位机通常需要欧拉角 (Roll, Pitch, Yaw - 直观理解)
- 偏航角用于陀螺仪补偿和云台控制

---

## 🔌 串口通信配置

### 串口初始化

```cpp
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<USBCommunicationNode>("sentry_communication");
    
    try
    {
        // 打开 USB 串口
        ros_serial_ptr = std::make_shared<serial::Serial>(
            USB_PORT,                                    // 设备路径 (e.g. "/dev/ttyUSB0")
            115200,                                      // 波特率
            serial::Timeout::simpleTimeout(100)         // 超时 100ms
        );
    }
    catch(serial::IOException &e)
    {
        RCLCPP_ERROR(node_ptr->get_logger(), "unable to open:%s", e.what());
        return -1;
    }
    
    // 验证串口状态
    if (ros_serial_ptr->isOpen())
    {
        RCLCPP_INFO(node_ptr->get_logger(), "SUCCEED OPEN %s", USB_PORT);
    }
    else
    {
        return -1;
    }
    
    // 使用多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_ptr);
    executor.spin();
    
    // 清理资源
    rclcpp::shutdown();
    ros_serial_ptr->close();
    return 0;
}
```

### 串口参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 波特率 | 115200 | 通信速度 (bits/second) |
| 数据位 | 8 | 每个字符 8 位 |
| 停止位 | 1 | 数据帧末尾停止位 |
| 校验位 | 无 | 不进行奇偶校验 |
| 超时 | 100ms | 无数据时的等待时间 |

---

## ⏱️ 通信时序

### 定时器工作周期

```
时间轴:
|-------|-------|-------|-------|-------|
  2ms     2ms     2ms     2ms     2ms    

每个 2ms 执行一次:
1. 发送当前命令数据到下位机
2. 读取下位机的反馈数据
3. 发布解析后的 ROS2 消息
4. 检查超时计数器
5. 更新错误状态
```

**通信频率:** **500 Hz** (每秒 500 次)

**时序图:**

```
cmd_vel 话题发布  ──────────────────────────────
                  │
                  ▼
USB 节点接收    ┌───┐
                │   │
                └───┘
                  │
                  ▼ (存储到缓冲区)
              每 2ms 定时器
                  │
                  ▼
          发送给下位机 (串口)
                  │
                  ├─ 数据 ──→ 下位机处理
                  │
                  └─ 等待反馈
                     │
                     ▼ (返回)
          接收下位机数据
                  │
                  ▼
          解析并发布 ROS2 话题
```

---

## 🔗 多线程架构

### 回调组 (Callback Group)

```cpp
// 使用 Reentrant 回调组,允许并发处理
callbackgroup = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant
);

auto sub_options = rclcpp::SubscriptionOptions();
sub_options.callback_group = callbackgroup;

// 所有订阅都使用此回调组
sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10, 
    std::bind(&USBCommunicationNode::sub_cmd_vel_callback, this, std::placeholders::_1), 
    sub_options
);
```

**为什么需要多线程?**
- 多个话题并发发布
- 定时器需要准时执行
- 避免某个回调阻塞整个系统

---

## 📋 节点参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `USB_PORT` | string | - | USB 串口设备路径 (宏定义) |
| 波特率 | int | 115200 | 串口波特率 |
| 定时器周期 | int | 2ms | 发送/接收数据的周期 |
| 超时阈值 | int | 250 | 超时计数阈值 (250 * 2ms = 500ms) |

---

## 🚀 使用示例

### 启动节点

```bash
# 方式 1: 直接启动
ros2 run sentry_control sentry_communication

# 方式 2: 通过启动文件
ros2 launch rm_nav_bringup bringup_real.launch.py
```

### 监控通信状态

```bash
# 查看发送的命令
ros2 topic echo /cmd_vel

# 查看接收的反馈
ros2 topic echo /SerialReciveData

# 查看裁判系统信息
ros2 topic echo /referee

# 查看 IMU 数据
ros2 topic echo imu/data

# 查看云台命令
ros2 topic echo /GimbalCmd
```

### 调试串口通信

```bash
# 方式 1: 使用 ROS2 日志查看详细信息
ros2 launch sentry_control sentry_communication --log-level sentry_communication:=DEBUG

# 方式 2: 监视串口数据 (需要 screen/minicom)
screen /dev/ttyUSB0 115200

# 方式 3: 检查串口设备
ls -la /dev/tty*
```

---

## 🐛 常见问题排查

### 问题 1: 串口打开失败

**错误信息:**
```
[ERROR] unable to open: Permission denied
```

**解决方案:**
```bash
# 添加用户到 dialout 组
sudo usermod -a -G dialout $USER

# 刷新用户组 (需要重新登录或运行)
newgrp dialout

# 检查权限
ls -l /dev/ttyUSB0
```

### 问题 2: 机器人响应延迟

**原因分析:**
- 串口波特率过低
- 定时器周期过长
- 数据包过大导致传输时间长

**优化方法:**
```cpp
// 增加波特率
ros_serial_ptr = std::make_shared<serial::Serial>(
    USB_PORT,
    921600,  // 从 115200 提升到 921600
    serial::Timeout::simpleTimeout(50)
);

// 减少定时器周期 (需要谨慎)
timer = this->create_wall_timer(
    std::chrono::milliseconds(1),  // 从 2ms 改为 1ms
    std::bind(&USBCommunicationNode::timer_callback, this),
    callbackgroup
);
```

### 问题 3: 机器人无法停止 (看门狗失效)

**可能原因:**
- 导航系统宕机,无法发送 /cmd_vel
- 超时检测逻辑有问题

**调试方法:**
```bash
# 查看 /cmd_vel 是否继续发布
ros2 topic echo --once /cmd_vel

# 检查节点状态
ros2 node list
ros2 node info /sentry_communication
```

---

## 📚 依赖包

```xml
<!-- package.xml 中的依赖 -->
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>serial</depend>
<depend>auto_aim_interfaces</depend>
<depend>sentry_nav_decision_interface</depend>
```

---

## 🔐 安全建议

1. **定期检查串口连接**
   - 确保 USB 线缆完好
   - 定期重新插拔以清除接触不良

2. **监控命令超时**
   - 在导航系统启动时启动本节点
   - 避免单独启动此节点而导航系统未运行

3. **错误日志记录**
   - 启用 DEBUG 日志级别监控通信
   - 记录异常断开连接事件

4. **硬件保护**
   - 确认下位机正确供电
   - 避免瞬间断电或复位

---

## 📖 相关文件

| 文件 | 功能 |
|------|------|
| `sentry_communication.cpp` | 主程序实现 |
| `sentry_communication.hpp` | 头文件,数据结构定义 |
| `CMakeLists.txt` | 编译配置 |
| `package.xml` | 功能包元数据 |

---

## 🎯 总结

| 功能 | 说明 |
|------|------|
| **核心职能** | 上位机(ROS2)与下位机(硬件)通信桥梁 |
| **通信方式** | USB 串口,115200 波特率 |
| **通信频率** | 500 Hz (每 2ms 一次) |
| **数据流向** | 双向 (发送命令 + 接收反馈) |
| **安全机制** | 500ms 命令超时自动停止 |
| **线程模型** | 多线程执行器,并发处理 |

---

## 📞 维护信息

- **最后更新**: 2025-10-20
- **维护者**: Sentry 项目组
- **文档版本**: 1.0.0

---

## 📄 许可证

根据项目许可证,此功能包遵循相应规则。

