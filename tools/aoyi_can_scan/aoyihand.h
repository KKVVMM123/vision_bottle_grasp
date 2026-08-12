#ifndef AOYIHAND_H
#define AOYIHAND_H
#include <stdint.h>
#include <vector>
#include <numeric>
#include "serial/serial.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <unistd.h>
#include <thread>
#include <csignal>

#include <fcntl.h>
#include <string>
#include <memory>

#include <limits.h>  // 添加这个头文件以定义 PATH_MAX
#include <stdlib.h> 
#include <libusb-1.0/libusb.h>
#include "Ti5_socketcan.h"

using namespace std;

enum aoyi_hand
{
    right_a,
    left_a
};

// 协议定义
#define PROTOCOL_HEADER_1 0x55
#define PROTOCOL_HEADER_2 0xAA

// 新增函数：构建完整的串口协议数据包
vector<uint8_t> build_aoyi_packet(uint16_t values[6], uint8_t speed[6], aoyi_hand side,int *right,int *left);


// 新增函数：通过CAN发送完整串口协议数据包
bool send_aoyi_packet_via_can(const vector<uint8_t> &packet, aoyi_hand side,int *right,int *left);


// 新增函数：通过CAN接收响应（模拟串口响应）
vector<uint8_t> receive_aoyi_response_via_can(aoyi_hand side ,int *right,int *left,int timeout_ms = 3);


// 辅助函数：检查是否收到完整响应（需要根据实际协议实现）
bool is_complete_response(const vector<uint8_t> &response);


// 主函数：完整的CAN方式角度控制（发送完整串口协议数据包）
void aoyijiaodu_via_can(uint16_t values[6], uint8_t speed[6], aoyi_hand side,int *right,int *left);


// 响应验证函数（需要实现）
bool validate_response(const vector<uint8_t> &response);


// 需要实现的calculateLRC函数（如果还没有）
uint8_t calculateLRC(const uint8_t *data, size_t length);

vector<uint8_t> create_status_command(aoyi_hand side, int *right, int *left);




#endif
