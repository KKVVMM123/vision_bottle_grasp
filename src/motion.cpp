#include "motion.h"
#include "aoyihand.h"
#include "serial/serial.h"

#include <fstream>
#include <sstream>

static serial::Serial *g_hand_serial = nullptr;

bool loadGraspConfig(const std::string &path, GraspConfig &cfg)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "[motion] 无法打开配置文件: " << path << "，使用默认参数" << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line))
    {
        // 去掉首尾空白
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos)
            continue;
        size_t end = line.find_last_not_of(" \t\r\n");
        line = line.substr(start, end - start + 1);
        if (line.empty() || line[0] == '#')
            continue;

        size_t eq = line.find('=');
        if (eq == std::string::npos)
            continue;

        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        val.erase(0, val.find_first_not_of(" \t"));
        val.erase(val.find_last_not_of(" \t") + 1);

        try
        {
            if (key == "tcp_host")
                cfg.tcp_host = val;
            else if (key == "tcp_port")
                cfg.tcp_port = std::stoi(val);
            else if (key == "head_push_interval_ms")
                cfg.head_push_interval_ms = std::stoi(val);
            else if (key == "right_hand_can_channel")
                cfg.right_hand_can_channel = std::stoi(val);
            else if (key == "right_hand_can_id")
                cfg.right_hand_can_id = std::stoi(val);
            else if (key == "left_hand_can_channel")
                cfg.left_hand_can_channel = std::stoi(val);
            else if (key == "left_hand_can_id")
                cfg.left_hand_can_id = std::stoi(val);
            else if (key == "approach_x_offset_m")
                cfg.approach_x_offset_m = std::stod(val);
            else if (key == "grasp_depth_offset_m")
                cfg.grasp_depth_offset_m = std::stod(val);
            else if (key == "grasp_y_offset_m")
                cfg.grasp_y_offset_m = std::stod(val);
            else if (key == "hand_open_values")
                cfg.hand_open_values = val;
            else if (key == "hand_close_values")
                cfg.hand_close_values = val;
            else if (key == "hand_hold_values")
                cfg.hand_hold_values = val;
            else if (key == "grasp_yaw_deg")
                cfg.grasp_yaw_deg = std::stod(val);
            else if (key == "lift_height_m")
                cfg.lift_height_m = std::stod(val);
            else if (key == "retreat_x_m")
                cfg.retreat_x_m = std::stod(val);
            else if (key == "retreat_y_m")
                cfg.retreat_y_m = std::stod(val);
            else if (key == "retreat_z_m")
                cfg.retreat_z_m = std::stod(val);
            else if (key == "move_velocity_mps")
                cfg.move_velocity_mps = std::stod(val);
            else if (key == "approach_velocity_mps")
                cfg.approach_velocity_mps = std::stod(val);
            else if (key == "release_clearance_height_m")
                cfg.release_clearance_height_m = std::stod(val);
            else if (key == "release_shift_y_m")
                cfg.release_shift_y_m = std::stod(val);
            else if (key == "hand_release_interval_ms")
                cfg.hand_release_interval_ms = std::stoi(val);
            else if (key == "hand_release_speed")
                cfg.hand_release_speed = std::stoi(val);
            else if (key == "dry_run")
                cfg.dry_run = (std::stoi(val) != 0);
        }
        catch (...)
        {
            std::cerr << "[motion] 配置解析失败: " << line << std::endl;
        }
    }
    return true;
}

void readHeadAnglesDeg(double &roll_deg, double &pitch_deg, double &yaw_deg)
{
    // 与商超 main3.cpp appendHeadAnglesToRequest 一致；读失败重试并用上次有效值兜底
    static double last_roll = 0.0, last_pitch = 0.0, last_yaw = 0.0;
    static bool have_last = false;
    const double to_deg = 360.0 / (65536.0 * 4.0);
    uint32_t hand_canID[3] = {30, 31, 32};

    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt)
    {
        int head_pulses[3] = {0, 0, 0};
        socketcan_sendsimplecommand(head_can_id, 3, hand_canID, 8, head_pulses);
        if (head_pulses[0] != 0 || head_pulses[1] != 0 || head_pulses[2] != 0)
        {
            roll_deg = head_pulses[0] * to_deg;
            pitch_deg = head_pulses[1] * to_deg;
            yaw_deg = head_pulses[2] * to_deg;
            last_roll = roll_deg;
            last_pitch = pitch_deg;
            last_yaw = yaw_deg;
            have_last = true;
            ok = true;
        }
    }

    if (!ok)
    {
        if (have_last)
        {
            roll_deg = last_roll;
            pitch_deg = last_pitch;
            yaw_deg = last_yaw;
            std::cerr << "[motion] 头部角读取失败，使用上次值(" << roll_deg << "," << pitch_deg << "," << yaw_deg << ")" << std::endl;
        }
        else
        {
            roll_deg = 0.0;
            pitch_deg = 0.0;
            yaw_deg = 0.0;
            std::cerr << "[motion] 头部角读取失败，无上次值可用" << std::endl;
        }
    }
}

int moveL(Robot &arm, const Matrix<double, 1, 6> &posd_in, double speed)
{
    Matrix<double, 1, 6> posd = posd_in; // 内部可能被 IK 修改
    usleep(250000);
    Matrix<double, 1, 7> q_current = arm.getJointPos();
    int ret = arm.getOPt_IK(posd, q_current);
    if (ret < 0)
    {
        std::cout << "[motion] IK 失败 ret=" << ret << std::endl;
        std::cout << "[motion] q_current(deg): " << q_current * deg << std::endl;
        return ret;
    }
    ret = arm.moveLToPosIK(posd, speed, q_current[1]);
    return ret;
}

int moveRightToReady(Robot &arm)
{
    Matrix<double, 1, 7> q_s_j_r;
    q_s_j_r << -0.761221, -0.703564, 0, 1.290989, 0, -0.0, 0.0;
    std::cout << "[motion] 回到右臂预备姿态" << std::endl;
    return arm.moveJtoJoint(q_s_j_r);
}

bool handSerialOpen(const GraspConfig &cfg)
{
    if (g_hand_serial != nullptr && g_hand_serial->isOpen())
        return true;
    try
    {
        g_hand_serial = new serial::Serial(cfg.hand_serial_port, cfg.hand_serial_baud,
                                           serial::Timeout::simpleTimeout(500));
        if (!g_hand_serial->isOpen())
        {
            delete g_hand_serial;
            g_hand_serial = nullptr;
            std::cerr << "[hand] 打开串口失败: " << cfg.hand_serial_port << std::endl;
            return false;
        }
        std::cout << "[hand] 串口已打开: " << cfg.hand_serial_port
                  << " id=" << cfg.hand_serial_id << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[hand] 打开串口异常: " << e.what() << std::endl;
        return false;
    }
}

bool handSerialSend(const GraspConfig &cfg, uint8_t command, const std::vector<uint8_t> &data)
{
    if (!handSerialOpen(cfg))
        return false;
    std::vector<uint8_t> pkt;
    pkt.push_back(0x55);
    pkt.push_back(0xAA);
    pkt.push_back(static_cast<uint8_t>(cfg.hand_serial_id));
    pkt.push_back(0x01); // master
    pkt.push_back(command);
    pkt.push_back(static_cast<uint8_t>(data.size()));
    pkt.insert(pkt.end(), data.begin(), data.end());
    uint8_t lrc = 0;
    for (size_t i = 2; i < pkt.size(); ++i)
        lrc ^= pkt[i];
    pkt.push_back(lrc);
    try
    {
        g_hand_serial->write(pkt);
        std::vector<uint8_t> buf(128);
        g_hand_serial->read(buf.data(), buf.size());
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[hand] 串口发送异常: " << e.what() << std::endl;
        return false;
    }
}

// 组装 6 路角度指令数据（value uint16 原始值 + speed）
static std::vector<uint8_t> buildAngleData(const uint16_t values[6], uint8_t speed)
{
    std::vector<uint8_t> data;
    for (int i = 0; i < 6; ++i)
    {
        data.push_back(values[i] & 0xFF);
        data.push_back((values[i] >> 8) & 0xFF);
        data.push_back(speed);
    }
    return data;
}

// 解析 "v0,v1,v2,v3,v4,v5" 到 values[6]
static bool parseUint16List(const std::string &text, uint16_t values[6])
{
    std::stringstream ss(text);
    std::string item;
    int idx = 0;
    while (std::getline(ss, item, ','))
    {
        if (idx >= 6)
            return false;
        try
        {
            values[idx++] = static_cast<uint16_t>(std::stoul(item));
        }
        catch (...)
        {
            return false;
        }
    }
    return idx == 6;
}

void handOpen(const GraspConfig &cfg)
{
    // 本机 T170C：手走 USB 串口。张开(预抓)姿态默认 {0,0,0,0,0,65000}
    uint16_t vals[6] = {0, 0, 0, 0, 0, 65000};
    if (!parseUint16List(cfg.hand_open_values, vals))
        std::cerr << "[hand] hand_open_values 解析失败，使用默认" << std::endl;
    handSerialSend(cfg, 0x50, buildAngleData(vals, 255));
    usleep(200000);
}

void handClose(const GraspConfig &cfg)
{
    // 闭合抓取默认 {30000,30000,30000,30000,30000,65000}
    uint16_t vals[6] = {30000, 30000, 30000, 30000, 30000, 65000};
    if (!parseUint16List(cfg.hand_close_values, vals))
        std::cerr << "[hand] hand_close_values 解析失败，使用默认" << std::endl;
    handSerialSend(cfg, 0x50, buildAngleData(vals, 255));
    usleep(200000);
}

// 抓稳后降力保持：用比闭合值小的 hand_hold_values 夹持，避免长时间全力压瓶导致堵转报警
void handHold(const GraspConfig &cfg)
{
    uint16_t vals[6] = {20000, 20000, 20000, 20000, 20000, 65000};
    if (!parseUint16List(cfg.hand_hold_values, vals))
        std::cerr << "[hand] hand_hold_values 解析失败，使用默认" << std::endl;
    handSerialSend(cfg, 0x50, buildAngleData(vals, 255));
    usleep(200000);
}

// 逐指松开：手指 4~0 依次由闭合值置为张开值，指 5 保持 65000 不变
// 每根手指间隔 hand_release_interval_ms，速度 hand_release_speed
void handReleaseSequential(const GraspConfig &cfg)
{
    uint16_t close_vals[6] = {30000, 30000, 30000, 30000, 30000, 65000};
    uint16_t open_vals[6] = {0, 0, 0, 0, 0, 65000};
    if (!parseUint16List(cfg.hand_close_values, close_vals))
        std::cerr << "[hand] hand_close_values 解析失败，使用默认" << std::endl;
    if (!parseUint16List(cfg.hand_open_values, open_vals))
        std::cerr << "[hand] hand_open_values 解析失败，使用默认" << std::endl;

    uint8_t speed = static_cast<uint8_t>(cfg.hand_release_speed & 0xFF);
    uint16_t vals[6];
    for (int i = 0; i < 6; ++i)
        vals[i] = close_vals[i];

    for (int i = 4; i >= 0; --i)
    {
        vals[i] = open_vals[i];
        if (cfg.dry_run)
        {
            std::cout << "[hand] dry_run 逐指松开: 手指 " << i << " -> " << open_vals[i] << std::endl;
            continue;
        }
        handSerialSend(cfg, 0x50, buildAngleData(vals, speed));
        usleep(static_cast<useconds_t>(cfg.hand_release_interval_ms) * 1000);
    }
}

bool rightHandGrasped(const GraspConfig &cfg)
{
    // 串口 0x5F 状态查询（子命令 0x80），6 路全 0x02 = 张开
    if (!handSerialOpen(cfg))
        return false;
    std::vector<uint8_t> data = {0x80};
    std::vector<uint8_t> pkt;
    pkt.push_back(0x55);
    pkt.push_back(0xAA);
    pkt.push_back(static_cast<uint8_t>(cfg.hand_serial_id));
    pkt.push_back(0x01);
    pkt.push_back(0x5F);
    pkt.push_back(static_cast<uint8_t>(data.size()));
    pkt.insert(pkt.end(), data.begin(), data.end());
    uint8_t lrc = 0;
    for (size_t i = 2; i < pkt.size(); ++i)
        lrc ^= pkt[i];
    pkt.push_back(lrc);

    try
    {
        g_hand_serial->flushInput();
        g_hand_serial->write(pkt);
        std::vector<uint8_t> raw(256);
        size_t n = g_hand_serial->read(raw.data(), raw.size());
        for (size_t i = 0; i + 6 < n; ++i)
        {
            if (raw[i] != 0x55 || raw[i + 1] != 0xAA)
                continue;
            uint8_t resp_hand = raw[i + 3];
            uint8_t resp_cmd = raw[i + 4] & 0x7F;
            uint8_t data_len = raw[i + 5];
            if (resp_hand != static_cast<uint8_t>(cfg.hand_serial_id) || resp_cmd != 0x5F)
                continue;
            if (data_len != 6 || i + 6 + data_len > n)
                continue;
            bool all_02 = true;
            for (int j = 0; j < 6; ++j)
            {
                if (raw[i + 6 + j] != 0x02)
                {
                    all_02 = false;
                    break;
                }
            }
            return !all_02;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[motion] 手状态查询异常: " << e.what() << std::endl;
    }
    std::cerr << "[motion] 未收到有效手状态响应" << std::endl;
    return false;
}

int executeGrasp(Robot &arm, const double goal_xyz[3], double yaw_deg, const GraspConfig &cfg,
                  double depth_offset_override)
{
    const double yaw_rad = yaw_deg * M_PI / 180.0;

    // 商超式：水平前伸抓取，姿态 RPY=[0,0,yaw]；抓取点加深度补偿（停在瓶身前表面）
    // depth_offset_override: 视觉按瓶径自动计算的补偿（0 表示未提供，用配置默认）
    const double depth_off = (depth_offset_override != 0.0) ? depth_offset_override
                                                            : cfg.grasp_depth_offset_m;
    Matrix<double, 1, 6> grasp_pose;
    grasp_pose << goal_xyz[0] + depth_off,
                  goal_xyz[1] + cfg.grasp_y_offset_m,
                  goal_xyz[2],
                  0, 0, yaw_rad;
    std::cout << "[motion] 深度补偿 depth_off=" << depth_off
              << "，y补偿=" << cfg.grasp_y_offset_m
              << "，实际抓取点 x=" << grasp_pose(0)
              << " y=" << grasp_pose(1) << std::endl;

    // staging 在目标前方（靠近机器人侧）
    Matrix<double, 1, 6> staging_pose = grasp_pose;
    staging_pose(0) -= cfg.approach_x_offset_m;

    Matrix<double, 1, 6> lift_pose = grasp_pose;
    lift_pose(2) += cfg.lift_height_m;

    Matrix<double, 1, 6> retreat_pose = lift_pose;
    retreat_pose(0) += cfg.retreat_x_m;
    retreat_pose(1) += cfg.retreat_y_m;
    retreat_pose(2) += cfg.retreat_z_m;

    // 工作空间检查：与 Robot::check_workspace_access 一致（法兰到 p2=(0,-0.192,0) 距离 <0.522）
    const double flange_x = goal_xyz[0] - 0.150;
    const double flange_y = goal_xyz[1] - 0.020;
    const double flange_z = goal_xyz[2];
    const double d26 = std::sqrt(flange_x * flange_x +
                                 (flange_y + 0.192) * (flange_y + 0.192) +
                                 flange_z * flange_z);
    std::cout << "[motion] 目标(" << goal_xyz[0] << "," << goal_xyz[1] << "," << goal_xyz[2]
              << ") 法兰距离=" << d26 << " m (工作空间上限 0.522)" << std::endl;
    if (d26 > 0.522)
        std::cerr << "[motion] 警告：超出右臂工作空间！请把瓶子放到 x≈0.35~0.40m、z≈-0.42m（桌面高度）"
                  << std::endl;
    if (goal_xyz[2] > 0.0)
        std::cerr << "[motion] 警告：目标 z=" << goal_xyz[2]
                  << " >0（按商超约定桌面在 z≈-0.4），请检查头部角/标定是否正确" << std::endl;

    std::cout << "[motion] 抓取计划:" << std::endl;
    std::cout << "  staging : " << staging_pose << std::endl;
    std::cout << "  grasp   : " << grasp_pose << std::endl;
    std::cout << "  lift    : " << lift_pose << std::endl;
    std::cout << "  retreat : " << retreat_pose << std::endl;

    if (cfg.dry_run)
    {
        std::cout << "[motion] dry_run=true，仅规划不执行" << std::endl;
        return 0;
    }

    // 1. 到 staging（手保持中性状态，避免张开的指头一路戳瓶子；对齐商超"接近时才张开"）
    int ret = moveL(arm, staging_pose, cfg.move_velocity_mps);
    if (ret < 0)
    {
        std::cerr << "[motion] staging MoveL 失败: " << ret << std::endl;
        moveRightToReady(arm);
        return -1;
    }

    // 2. 到 staging 后再张开(预抓)
    handOpen(cfg);
    usleep(300000);

    // 3. 慢速接近到目标（瓶身前表面）
    ret = moveL(arm, grasp_pose, cfg.approach_velocity_mps);
    if (ret < 0)
    {
        std::cerr << "[motion] 接近 MoveL 失败: " << ret << std::endl;
        moveRightToReady(arm);
        return -1;
    }

    // 4. 闭合抓取
    handClose(cfg);
    usleep(300000);

    // 5. 校验是否抓住
    bool grasped = rightHandGrasped(cfg);
    if (!grasped)
        std::cerr << "[motion] 可能未抓住（手状态异常）" << std::endl;

    // 6. 抬升
    ret = moveL(arm, lift_pose, cfg.move_velocity_mps);
    if (ret < 0)
    {
        std::cerr << "[motion] 抬升 MoveL 失败: " << ret << std::endl;
        moveRightToReady(arm);
        return -2;
    }

    // 7. 回退保持
    ret = moveL(arm, retreat_pose, cfg.move_velocity_mps);
    if (ret < 0)
    {
        std::cerr << "[motion] 回退 MoveL 失败: " << ret << std::endl;
        moveRightToReady(arm);
        return -3;
    }

    // 8. 抓稳后降力保持（防堵转）：抬升/回退用全夹紧力，到位后再放松到保持值
    if (!cfg.dry_run)
        handHold(cfg);
    else
        std::cout << "[motion] dry_run 跳过降力保持" << std::endl;

    std::cout << "[motion] 抓取完成，保持当前位置" << std::endl;
    return grasped ? 0 : 1;
}

// 放瓶序列失败回退：尽力抬高当前 TCP 到净空高度，再回预备位；
// 不强制张手，避免把瓶子摔落
static int releaseFallback(Robot &arm, const GraspConfig &cfg, int ret)
{
    std::cerr << "[motion] 放瓶序列失败(ret=" << ret << ")，回退：抬高->回预备位" << std::endl;
    try
    {
        Matrix<double, 1, 6> safe = arm.getTcpPos();
        safe(2) += cfg.release_clearance_height_m;
        moveL(arm, safe, cfg.move_velocity_mps);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[motion] 回退抬高异常: " << e.what() << std::endl;
    }
    moveRightToReady(arm);
    return ret;
}

int releaseHand(Robot &arm, const GraspConfig &cfg)
{
    // 放瓶计划：
    //   1) 放瓶：z 下降 lift_height_m（抬升量原样降回，瓶底贴桌面）
    //   2) 逐指松开：手指 4~0 依次张开
    //   3) 净空：直接从放瓶位 z 抬高 release_clearance_height_m，高于瓶顶
    //   4) 整臂外摆：沿 y 平移 release_shift_y_m，整臂协同摆开避让瓶/桌
    //   5) 回预备位
    Matrix<double, 1, 6> cur = arm.getTcpPos();
    Matrix<double, 1, 6> place_pose = cur;
    place_pose(2) -= cfg.lift_height_m;
    Matrix<double, 1, 6> clear_pose = place_pose;
    clear_pose(2) += cfg.release_clearance_height_m;
    Matrix<double, 1, 6> shift_pose = clear_pose;
    shift_pose(1) += cfg.release_shift_y_m;

    std::cout << "[motion] 放瓶计划:" << std::endl;
    std::cout << "  当前  : " << cur << std::endl;
    std::cout << "  放瓶(z-" << cfg.lift_height_m << "): " << place_pose << std::endl;
    std::cout << "  净空(z+" << cfg.release_clearance_height_m << "): " << clear_pose << std::endl;
    std::cout << "  外摆(y" << (cfg.release_shift_y_m >= 0 ? "+" : "")
              << cfg.release_shift_y_m << "): " << shift_pose << std::endl;

    if (cfg.dry_run)
    {
        std::cout << "[motion] dry_run=true，仅打印放瓶计划，不动作" << std::endl;
        return 0;
    }

    // 1. 放瓶（慢速下降）
    int ret = moveL(arm, place_pose, cfg.approach_velocity_mps);
    if (ret < 0)
    {
        std::cerr << "[motion] 放瓶下降 MoveL 失败: " << ret << std::endl;
        return releaseFallback(arm, cfg, ret);
    }

    // 2. 逐指松开
    handReleaseSequential(cfg);

    // 3. 直接从放瓶位抬高到净空（高于瓶顶）
    ret = moveL(arm, clear_pose, cfg.move_velocity_mps);
    if (ret < 0)
    {
        std::cerr << "[motion] 抬高 MoveL 失败: " << ret << std::endl;
        return releaseFallback(arm, cfg, ret);
    }

    // 4. 整臂外摆（笛卡尔横向平移，整臂协同摆开）
    ret = moveL(arm, shift_pose, cfg.move_velocity_mps);
    if (ret < 0)
    {
        std::cerr << "[motion] 整臂外摆 MoveL 失败: " << ret << std::endl;
        return releaseFallback(arm, cfg, ret);
    }

    // 5. 回预备位
    ret = moveRightToReady(arm);
    if (ret < 0)
        std::cerr << "[motion] 回预备位失败: " << ret << std::endl;

    std::cout << "[motion] 放瓶完成，已回预备位" << std::endl;
    return 0;
}
