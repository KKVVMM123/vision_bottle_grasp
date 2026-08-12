#include "aoyihand.h"
#include <chrono>
#include <iomanip>
#include <sstream>

namespace
{
constexpr uint8_t HAND_CMD_GET_PROTOCOL_VERSION = 0x00;
constexpr uint8_t HAND_CMD_GET_FW_VERSION = 0x01;
constexpr uint8_t HAND_CMD_GET_HW_VERSION = 0x02;
constexpr uint8_t HAND_CMD_GET_VENDOR_ID = 0x3F;
constexpr uint8_t MASTER_ID = 0x01;

vector<uint8_t> build_hand_read_packet(uint8_t hand_id, uint8_t command)
{
    vector<uint8_t> packet = {
        PROTOCOL_HEADER_1,
        PROTOCOL_HEADER_2,
        hand_id,
        MASTER_ID,
        command,
        0};
    packet.push_back(calculateLRC(packet.data() + 2, 4));
    return packet;
}

bool send_packet_raw_on_can(int channel, uint32_t can_id, const vector<uint8_t> &packet)
{
    if (channel < 0 || channel >= CAN_CHANNEL_COUNT || can_sockets[channel] < 0)
        return false;

    size_t sent_bytes = 0;
    while (sent_bytes < packet.size())
    {
        uint8_t frame_data[8] = {0};
        uint8_t dlc = 0;
        for (int i = 0; i < 8 && sent_bytes < packet.size(); i++)
        {
            frame_data[i] = packet[sent_bytes++];
            dlc++;
        }
        if (!send_can_frame(channel, can_id, frame_data, dlc))
            return false;
        usleep(1000);
    }
    return true;
}

void flush_can_channel_rx(int channel)
{
    uint8_t dummy[8];
    for (int i = 0; i < 32; i++)
    {
        if (!receive_can_frame_timeout(channel, dummy))
            break;
    }
}

vector<uint8_t> receive_can_bytes_idle(int channel, int idle_timeout_ms, int max_wait_ms)
{
    vector<uint8_t> full_response;
    auto start = chrono::steady_clock::now();
    auto last_data = start;

    while (true)
    {
        uint8_t frame_data[8] = {0};
        if (receive_can_frame_timeout(channel, frame_data))
        {
            for (int i = 0; i < 8; i++)
                full_response.push_back(frame_data[i]);
            last_data = chrono::steady_clock::now();
        }
        else
        {
            auto now = chrono::steady_clock::now();
            if (!full_response.empty() &&
                chrono::duration_cast<chrono::milliseconds>(now - last_data).count() >= idle_timeout_ms)
                break;
            if (chrono::duration_cast<chrono::milliseconds>(now - start).count() >= max_wait_ms)
                break;
            usleep(500);
        }
    }
    return full_response;
}

bool extract_hand_response_frame(const vector<uint8_t> &raw,
                                 uint8_t expected_hand_id,
                                 uint8_t expected_cmd,
                                 vector<uint8_t> &frame)
{
    for (size_t i = 0; i + 6 < raw.size(); i++)
    {
        if (raw[i] != PROTOCOL_HEADER_1 || raw[i + 1] != PROTOCOL_HEADER_2)
            continue;

        uint8_t hand_id = raw[i + 3];
        uint8_t cmd = raw[i + 4];
        uint8_t data_len = raw[i + 5];
        size_t frame_len = 6 + data_len + 1;
        if (i + frame_len > raw.size())
            continue;

        const uint8_t *payload = raw.data() + i;
        if (hand_id != expected_hand_id)
            continue;
        if ((cmd & 0x7F) != expected_cmd || (cmd & 0x80))
            continue;
        if (calculateLRC(payload + 2, 4 + data_len) != payload[6 + data_len])
            continue;

        frame.assign(payload, payload + frame_len);
        return true;
    }
    return false;
}

string describe_hand_probe_response(uint8_t cmd, const vector<uint8_t> &frame)
{
    if (frame.size() < 7)
        return "无效响应";

    uint8_t data_len = frame[5];
    if (frame.size() < 6u + data_len)
        return "响应数据不完整";

    const uint8_t *data = frame.data() + 6;
    ostringstream oss;

    switch (cmd)
    {
    case HAND_CMD_GET_PROTOCOL_VERSION:
        if (data_len >= 2)
            oss << "协议版本 v" << static_cast<int>(data[1]) << "." << static_cast<int>(data[0]);
        break;
    case HAND_CMD_GET_FW_VERSION:
        if (data_len >= 4)
            oss << "固件版本 v" << static_cast<int>(data[3]) << "." << static_cast<int>(data[2])
                << " rev" << (static_cast<uint16_t>(data[1]) << 8 | data[0]);
        break;
    case HAND_CMD_GET_HW_VERSION:
        if (data_len >= 4)
            oss << "硬件版本 type=" << static_cast<int>(data[0])
                << " hw=" << static_cast<int>(data[1])
                << " boot=" << static_cast<int>(data[2]) << "." << static_cast<int>(data[3]);
        break;
    case HAND_CMD_GET_VENDOR_ID:
        if (data_len >= 2)
        {
            oss << "供应商ID ";
            for (uint8_t j = 0; j < data_len; j++)
                oss << static_cast<char>(data[j]);
        }
        break;
    default:
        oss << "命令0x" << hex << static_cast<int>(cmd) << dec << " 数据长度=" << static_cast<int>(data_len);
        break;
    }

    if (oss.str().empty())
        return "收到有效响应";
    return oss.str();
}

void print_hex_bytes(const vector<uint8_t> &bytes)
{
    for (uint8_t b : bytes)
        cout << hex << setw(2) << setfill('0') << static_cast<int>(b) << " ";
    cout << dec << setfill(' ');
}
} // namespace

//-----------------------------------------------------------

// 新增函数：构建完整的串口协议数据包
vector<uint8_t> build_aoyi_packet(uint16_t values[6], uint8_t speed[6], aoyi_hand side, int *right, int *left)
{
    uint8_t handID = 0;
    if (side == right_a)
    {
        handID = right[1];
    }
    else
    {
        handID = left[1];
    }
    uint8_t masterID = 0x01;
    uint8_t command = 0x50;

    // 准备数据（与原有逻辑完全相同）
    for (int i = 0; i < 6; i++)
    {
        float temp = static_cast<float>(values[i]) / 90.0f;
        values[i] = static_cast<uint16_t>(temp * 64000.0f);
        if (values[i] > 65535)
            values[i] = 65535;
    }

    vector<uint8_t> data;
    for (int i = 0; i < 6; i++)
    {
        data.push_back(values[i] & 0xFF);        // 低字节
        data.push_back((values[i] >> 8) & 0xFF); // 高字节
        data.push_back(speed[i]);                // 速度值
    }

    // 构建完整数据包（包括包头、校验和等）
    vector<uint8_t> packet;
    packet.push_back(PROTOCOL_HEADER_1); // 假设这些常量已定义
    packet.push_back(PROTOCOL_HEADER_2);
    packet.push_back(handID);
    packet.push_back(masterID);
    packet.push_back(command);
    packet.push_back(static_cast<uint8_t>(data.size()));
    packet.insert(packet.end(), data.begin(), data.end());

    // 计算校验和（需要实现calculateLRC函数）
    uint8_t checksum = calculateLRC(packet.data() + 2, packet.size() - 2);
    packet.push_back(checksum);

    return packet;
}

// 新增函数：通过CAN发送完整串口协议数据包
bool send_aoyi_packet_via_can(const vector<uint8_t> &packet, aoyi_hand side, int *right, int *left)
{
    int channel = 0;
    uint32_t can_id = 2; // 使用CAN ID 2

    // 根据左右手选择CAN通道
    if (side == right_a)
    {
        channel = right[0];
        can_id = right[1];
    }
    else
    {
        channel = left[0];
        can_id = left[1];
    }

    // cout << "通过CAN发送完整数据包，大小: " << packet.size() << " 字节" << endl;
    // cout << "数据包内容: ";
    //  for (size_t i = 0; i < packet.size(); i++) {
    //      cout << "0x" << hex << (int)packet[i] << " ";
    //  }
    //  cout << dec << endl;

    // 数据分帧发送（每帧最多8字节）
    size_t total_size = packet.size();
    size_t sent_bytes = 0;
    uint8_t frame_count = 0;

    while (sent_bytes < total_size)
    {
        uint8_t frame_data[8] = {0};
        uint8_t dlc = 0;

        // 填充当前帧数据
        for (int i = 0; i < 8 && sent_bytes < total_size; i++)
        {
            frame_data[i] = packet[sent_bytes];
            sent_bytes++;
            dlc++;
        }

        // 发送CAN帧
        if (!send_can_frame(channel, can_id, frame_data, dlc))
        {
            cerr << "Failed to send CAN frame " << (int)frame_count << endl;
            return false;
        }

        // cout << "发送CAN帧 " << (int)frame_count << ": ";
        //  for (int i = 0; i < dlc; i++) {
        //      cout << "0x" << hex << (int)frame_data[i] << " ";
        //  }
        //  cout << dec << endl;

        frame_count++;
        usleep(1000); // 小延迟避免总线过载
    }

    // cout << "完整数据包发送完成，共 " << (int)frame_count << " 帧" << endl;
    return true;
}

// // 新增函数：通过CAN接收响应（模拟串口响应）
// vector<uint8_t> receive_aoyi_response_via_can(aoyi_hand side,int *right,int *left ,int timeout_ms)
// {
//     int channel = 0;
//     vector<uint8_t> full_response;

//     if (side == right_a) {
//         channel = right[0];
//     } else {
//         channel = left[0];
//     }

//     auto start_time = chrono::steady_clock::now();

//     //cout << "开始接收CAN响应..." << endl;

//     // 持续读取直到超时
//     while (true) {
//         uint8_t frame_data[8];

//         if (receive_can_frame_timeout(channel, frame_data)) {
//             // 将接收到的数据添加到完整响应中
//             for (int i = 0; i < 8; i++) {
//                 full_response.push_back(frame_data[i]);
//             }

//             //cout << "收到CAN响应帧: ";
//             // for (int i = 0; i < 8; i++) {
//             //     cout << "0x" << hex << (int)frame_data[i] << " ";
//             // }
//             // cout << dec << endl;

//             // 重置超时计时
//             start_time = chrono::steady_clock::now();

//             // 检查是否收到完整响应（根据协议判断）
//             if (is_complete_response(full_response)) {
//                 //cout << "收到完整响应，大小: " << full_response.size() << " 字节" << endl;
//                 break;
//             }
//         }

//         // 检查超时
//         auto current_time = chrono::steady_clock::now();
//         auto elapsed = chrono::duration_cast<chrono::milliseconds>(current_time - start_time).count();

//         if (elapsed > timeout_ms) {
//             //cout << "接收响应超时" << endl;
//             break;
//         }

//         //usleep(1000);
//     }

//     return full_response;
// }

// 修改后的函数：直接读取CAN数据，不进行完整性检查
vector<uint8_t> receive_aoyi_response_via_can(aoyi_hand side, int *right, int *left, int timeout_ms)
{
    int channel = 0;
    vector<uint8_t> full_response;

    if (side == right_a)
    {
        channel = right[0];
    }
    else
    {
        channel = left[0];
    }

    // 持续读取直到超时
    while (true)
    {

        uint8_t frame_data[8]={};

        // 尝试读取一帧CAN数据
        if (receive_can_frame_timeout(channel, frame_data))
        {
            // // 将接收到的8字节数据添加到完整响应中
            for (int i = 0; i < 8; i++)
            {
               // cout << "frame_data" << i << ": 0x" << hex << (int)frame_data[i] << endl;
                full_response.push_back(frame_data[i]);
            }
        }
        else
        {
            break;
        }
        usleep(100);
    }

    // cout << "CAN响应接收完成，总共收到 " << full_response.size() << " 字节" << endl;
    return full_response;
}

// 辅助函数：检查是否收到完整响应（需要根据实际协议实现）
bool is_complete_response(const vector<uint8_t> &response)
{
    // 这里需要根据您的协议来判断响应是否完整
    // 例如：检查包头、数据长度等
    if (response.size() < 6)
        return false; // 至少需要包头等基本信息

    // 示例：检查包头
    if (response[0] != PROTOCOL_HEADER_1 || response[1] != PROTOCOL_HEADER_2)
    {
        return false;
    }

    // 根据数据长度字段判断是否完整
    if (response.size() >= 6)
    {
        uint8_t data_len = response[5];
        uint8_t expected_size = 6 + data_len + 1; // 包头6字节 + 数据 + 校验和
        return response.size() >= expected_size;
    }

    return false;
}

// 主函数：完整的CAN方式角度控制（发送完整串口协议数据包）
void aoyijiaodu_via_can(uint16_t values[6], uint8_t speed[6], aoyi_hand side, int *right, int *left)
{
    // cout << "=== 通过CAN发送灵巧手角度指令 ===" << endl;

    // 1. 构建完整的串口协议数据包
    vector<uint8_t> packet = build_aoyi_packet(values, speed, side, right, left);

    // 2. 通过CAN发送完整数据包
    if (!send_aoyi_packet_via_can(packet, side, right, left))
    {
        cerr << "发送数据包失败" << endl;
        return;
    }

    // // 3. 读取响应
    // vector<uint8_t> response = receive_aoyi_response_via_can(side);

    // // 4. 处理响应
    // if (!response.empty()) {
    //     //cout << "收到响应，大小: " << response.size() << " 字节" << endl;

    //     // 验证响应（可选）
    //     if (validate_response(response)) {
    //         //cout << "响应验证成功" << endl;
    //     } else {
    //         //cerr << "响应验证失败" << endl;
    //     }
    // } else {
    //    // cerr << "未收到响应" << endl;
    // }
}

// 响应验证函数（需要实现）
bool validate_response(const vector<uint8_t> &response)
{
    // 实现响应验证逻辑，包括校验和验证等
    if (response.size() < 7)
        return false; // 至少需要6字节包头+1字节校验和

    // 校验和验证
    uint8_t received_checksum = response.back();
    uint8_t calculated_checksum = calculateLRC(response.data() + 2, response.size() - 3);

    return received_checksum == calculated_checksum;
}

vector<uint8_t> create_status_command(aoyi_hand side, int *right, int *left)
{
    uint8_t handID = (side == right_a) ? right[1] : left[1];
    uint8_t masterID = 0x01;
    uint8_t command = 0x5F;

    // 数据区：只包含子命令掩码 SUB_CMD_GET_STATUS = 0x80
    vector<uint8_t> data;
    data.push_back(0x80); // 子命令掩码
    uint8_t dataLen = data.size();

    // 构建完整数据包
    vector<uint8_t> packet;

    // 包头
    packet.push_back(0x55); // PROTOCOL_HEADER_1
    packet.push_back(0xAA); // PROTOCOL_HEADER_2

    // 设备ID和主机ID
    packet.push_back(handID);
    packet.push_back(masterID);

    // 命令码和数据长度
    packet.push_back(command);
    packet.push_back(dataLen); // 数据长度

    // 数据区
    packet.insert(packet.end(), data.begin(), data.end());

    // 标准LRC校验：计算4个固定字段 + 数据区
    uint8_t lrc = 0;
    uint8_t cmd_data_len = 4 + dataLen; // 设备ID+主机ID+命令码+数据长度+数据区
    
    for (int i = 0; i < cmd_data_len; i++) {
        lrc ^= packet[2 + i]; // 从索引2（设备ID）开始计算
    }
    packet.push_back(lrc);

    return packet;
}

vector<AoyiHandCanMatch> discover_aoyi_hand_can(int idle_timeout_ms, int max_wait_ms)
{
    const uint8_t candidate_hand_ids[] = {2, 3, 60, 70};
    const uint8_t probe_cmds[] = {
        HAND_CMD_GET_VENDOR_ID,
        HAND_CMD_GET_FW_VERSION,
        HAND_CMD_GET_PROTOCOL_VERSION,
        HAND_CMD_GET_HW_VERSION};

    vector<AoyiHandCanMatch> matches;

    cout << "\n=== 傲意灵巧手 CAN 通道/ID 扫描 ===" << endl;
    cout << "候选 HandID: 2, 3, 60, 70" << endl;
    cout << "扫描 CAN 通道: can0 ~ can" << (CAN_CHANNEL_COUNT - 1) << endl;

    for (int channel = 0; channel < CAN_CHANNEL_COUNT; channel++)
    {
        if (can_sockets[channel] < 0)
        {
            cout << "[跳过] can" << channel << " 未初始化" << endl;
            continue;
        }

        for (uint8_t hand_id : candidate_hand_ids)
        {
            bool hand_found_on_channel = false;

            for (uint8_t cmd : probe_cmds)
            {
                flush_can_channel_rx(channel);

                vector<uint8_t> packet = build_hand_read_packet(hand_id, cmd);
                if (!send_packet_raw_on_can(channel, hand_id, packet))
                    continue;

                usleep(3000);
                vector<uint8_t> raw = receive_can_bytes_idle(channel, idle_timeout_ms, max_wait_ms);

                vector<uint8_t> frame;
                if (!extract_hand_response_frame(raw, hand_id, cmd, frame))
                    continue;

                AoyiHandCanMatch match;
                match.can_channel = channel;
                match.hand_id = hand_id;
                match.probe_cmd = cmd;
                match.response = frame;
                match.info = describe_hand_probe_response(cmd, frame);
                matches.push_back(match);
                hand_found_on_channel = true;

                cout << "[发现] can" << channel
                     << "  HandID=" << static_cast<int>(hand_id)
                     << "  命令=0x" << hex << static_cast<int>(cmd) << dec
                     << "  " << match.info << endl;
                cout << "       响应: ";
                print_hex_bytes(frame);
                cout << endl;
            }

            if (hand_found_on_channel)
                cout << "[确认] can" << channel << " 上存在 HandID=" << static_cast<int>(hand_id) << endl;
        }
    }

    if (matches.empty())
        cout << "[结果] 未发现任何灵巧手响应，请检查 CAN 接线、波特率(1Mbps) 和供电" << endl;
    else
        cout << "[结果] 共发现 " << matches.size() << " 条有效响应" << endl;

    cout << "=== 扫描结束 ===\n" << endl;
    return matches;
}

void print_aoyi_hand_can_discovery(const vector<AoyiHandCanMatch> &matches)
{
    if (matches.empty())
    {
        cout << "未发现傲意灵巧手" << endl;
        return;
    }

    cout << "傲意灵巧手探测结果:" << endl;
    for (const auto &m : matches)
    {
        cout << "  can" << m.can_channel
             << "  id=" << static_cast<int>(m.hand_id)
             << "  cmd=0x" << hex << static_cast<int>(m.probe_cmd) << dec
             << "  " << m.info << endl;
    }
}



vector<uint8_t> receive_aoyi_response_quiet(aoyi_hand side, int *right, int *left)
{
    return receive_aoyi_response_via_can(side, right, left);
}

vector<uint8_t> all_receive_aoyi_response(aoyi_hand side, int *right, int *left)
{
    vector<uint8_t> response = receive_aoyi_response_via_can(side, right, left);

    if (!response.empty())
    {
        // cout << "=== 调试信息 ===" << endl;
        // cout << "response.size() = " << response.size() << endl;
        // cout << "实际元素个数: " << response.size() << endl;
        
        // // 方法1：逐个打印验证
        // cout << "逐个打印验证:" << endl;
 
        
        // // 方法2：使用printf避免格式问题
        // cout << "使用printf打印:" << endl;
        printf("十六进制格式: ");
        for (size_t i = 0; i < response.size(); ++i)
        {
            printf("%02X ", response[i]);
        }
        printf("\n");
        
        // 方法3：检查是否有内存越界
        //cout << "检查向量容量: " << response.capacity() << endl;
    }
    else
    {
        cout << "响应为空" << endl;
    }
    
    return response;
}


int tiger_can_hand(aoyi_hand side)
{
    int right_can_f[2] = {7, 60};
    int left_can_f[2] = {6, 70};

    uint16_t final_angles[6] = {0, 0, 0, 0, 0, 90};
    uint8_t speeds[6] = {255, 255, 255, 255, 255, 255};
    aoyijiaodu_via_can(final_angles, speeds, side, right_can_f, left_can_f);
    receive_aoyi_response_quiet(side, right_can_f, left_can_f);
    return 0;
}

int grasp_can_hand(aoyi_hand side)
{
    int right_can_f[2] = {7, 60};
    int left_can_f[2] = {6, 70};
    uint16_t final_angles[6] = {50, 41, 41, 41, 41, 90};
    uint8_t speeds[6] = {255, 255, 255, 255, 255, 255};
    aoyijiaodu_via_can(final_angles, speeds, side, right_can_f, left_can_f);
    receive_aoyi_response_quiet(side, right_can_f, left_can_f);
    return 0;
}

int release_can_hand(aoyi_hand side)
{
    int right_can_f[2] = {5, 3};
    int left_can_f[2] = {3, 2};
    uint16_t final_angles[6] = {0, 0, 0, 0, 0, 0};
    uint8_t speeds[6] = {255, 255, 255, 255, 255, 255};
    aoyijiaodu_via_can(final_angles, speeds, side, right_can_f, left_can_f);
    receive_aoyi_response_quiet(side, right_can_f, left_can_f);
    return 0;
}



int get_can_hand_status(aoyi_hand side)
{
    int right_can_f[2] = {5, 3};
    int left_can_f[2] = {3, 2};
    uint8_t hand_id = (side == right_a) ? static_cast<uint8_t>(right_can_f[1])
                                        : static_cast<uint8_t>(left_can_f[1]);

    vector<uint8_t> hand_po = create_status_command(side, right_can_f, left_can_f);
    if (!send_aoyi_packet_via_can(hand_po, side, right_can_f, left_can_f))
    {
        cerr << "发送数据包失败" << endl;
        return -1;
    }

    vector<uint8_t> raw = receive_aoyi_response_quiet(side, right_can_f, left_can_f);
    vector<uint8_t> frame;
    if (!extract_hand_response_frame(raw, hand_id, 0x5F, frame))
        return -1;

    if (frame.size() < 13 || frame[5] != 6)
        return -1;

    bool all_02 = true;
    for (int i = 0; i < 6; i++)
    {
        if (frame[6 + i] != 0x02)
        {
            all_02 = false;
            break;
        }
    }

    if (all_02)
        return -1;

    printf("十六进制格式: ");
    for (size_t i = 0; i < frame.size(); ++i)
        printf("%02X ", frame[i]);
    printf("\n");
    return 0;
}

