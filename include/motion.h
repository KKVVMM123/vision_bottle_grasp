#ifndef BOTTLE_GRASP_MOTION_H
#define BOTTLE_GRASP_MOTION_H

#include "head.h"

// 抓取运动配置（从 config/motion.txt 加载）
struct GraspConfig
{
    std::string tcp_host = "127.0.0.1";
    int tcp_port = 12345;
    int head_push_interval_ms = 500;   // C++ 主动推送头部角给视觉的间隔

    // 右手/左手灵巧手所在 CAN 通道与 ID（CAN 传输备用；本机 T170C 手走 USB 串口）
    int right_hand_can_channel = 7;
    int right_hand_can_id = 60;
    int left_hand_can_channel = 6;
    int left_hand_can_id = 70;

    // 灵巧手串口（本机 T170C：右手 /dev/ttyCH343USB1 id=60）
    std::string hand_serial_port = "/dev/ttyCH343USB1";
    int hand_serial_baud = 115200;
    int hand_serial_id = 60;

    // 抓取参数（商超式：水平前伸抓取）
    double approach_x_offset_m = 0.10;   // staging 点在目标前方（靠近机器人侧）的距离
    double grasp_depth_offset_m = -0.03; // 抓取点相对视觉重心的 x 补偿（负=停在瓶身前表面，避免手指撞瓶）
    double grasp_y_offset_m = 0.0;       // 抓取点相对视觉重心的 y 补偿（正=+y，负=-y；用于微调手对齐瓶身，避免手指碰瓶）
    double grasp_yaw_deg = 0.0;          // 抓取时腕部偏航角（度）
    double lift_height_m = 0.03;         // 抓取后抬升高度
    std::string hand_open_values = "0,0,0,0,0,65000";     // 手张开(预抓)姿态 6 路值
    std::string hand_close_values = "30000,30000,30000,30000,30000,65000"; // 手闭合抓取 6 路值
    std::string hand_hold_values = "20000,20000,20000,20000,20000,65000"; // 抓稳后降力保持 6 路值（防堵转）
    double retreat_x_m = -0.03;          // 回退向量 x
    double retreat_y_m = 0.0;            // 回退向量 y
    double retreat_z_m = 0.0;            // 回退向量 z
    double move_velocity_mps = 0.20;     // 常规 MoveL 速度
    double approach_velocity_mps = 0.10; // 接近目标速度

    // 放瓶/回预备位（按 h 或 HOME 时执行）
    double release_clearance_height_m = 0.12; // 放瓶后直接抬到放瓶高度之上的净空（应高于瓶顶）
    double release_shift_y_m = -0.05;    // 整臂向外摆开的横向位移（y 负向=机器人右侧），方向以 dry-run 实测为准
    int hand_release_interval_ms = 300;  // 逐指松开间隔
    int hand_release_speed = 255;        // 逐指松开速度（0~255）

    bool dry_run = false;                // true: 只打印规划，不动作
};

bool loadGraspConfig(const std::string &path, GraspConfig &cfg);

// 读取头部三轴当前角（度），与商超 main3 一致（ID 30/31/32）
void readHeadAnglesDeg(double &roll_deg, double &pitch_deg, double &yaw_deg);

// 右臂 MoveL（带 IK/工作空间检查），返回 0 成功，<0 失败
int moveL(Robot &arm, const Matrix<double, 1, 6> &posd, double speed);

// 右臂回到商超预备关节姿态
int moveRightToReady(Robot &arm);

// 右手张开（预抓姿态，角度与商超 tiger_can_hand 一致）
void handOpen(const GraspConfig &cfg);

// 右手闭合抓取（角度与商超 grasp_can_hand 一致）
void handClose(const GraspConfig &cfg);

// 抓稳后降力保持（防堵转）
void handHold(const GraspConfig &cfg);

// 串口手：打开串口并发送 OHand 指令（本机默认传输）
bool handSerialOpen(const GraspConfig &cfg);
bool handSerialSend(const GraspConfig &cfg, uint8_t command, const std::vector<uint8_t> &data);


// 查询右手是否抓到物体（0x5F 状态查询，6 路全 0x02 视为张开）
bool rightHandGrasped(const GraspConfig &cfg);

// 执行一次抓取：张开手 -> staging -> 接近 -> 闭合 -> 抬升 -> 回退保持
// goal_xyz: base 系目标 (x,y,z)；yaw_deg: 抓取腕部偏航角(度)
// 返回 0 成功；1 未抓住；<0 运动失败
int executeGrasp(Robot &arm, const double goal_xyz[3], double yaw_deg, const GraspConfig &cfg,
                double depth_offset_override = 0.0);

// 释放：张开手并回预备位
int releaseHand(Robot &arm, const GraspConfig &cfg);

#endif // BOTTLE_GRASP_MOTION_H
