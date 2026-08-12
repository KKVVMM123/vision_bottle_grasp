#include "aoyihand.h"

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
        values[i] = static_cast<uint16_t>(temp * 64535.0f);
        if (values[i] > 64535)
            values[i] = 64535;
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

// 需要实现的calculateLRC函数（如果还没有）
uint8_t calculateLRC(const uint8_t *data, size_t length)
{
    uint8_t lrc = 0;
    for (size_t i = 0; i < length; i++)
    {
        lrc ^= data[i];
    }
    return lrc;
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