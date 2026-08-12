#ifndef AOYIHAND_H
#define AOYIHAND_H
#include "aoyi_hand.h"
#include "Ti5_socketcan.h"

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


vector<uint8_t> create_status_command(aoyi_hand side, int *right, int *left);

struct AoyiHandCanMatch
{
    int can_channel = -1;
    uint8_t hand_id = 0;
    uint8_t probe_cmd = 0;
    vector<uint8_t> response;
    string info;
};

// 扫描所有 CAN 通道与候选 HandID，用读指令探测灵巧手位置
vector<AoyiHandCanMatch> discover_aoyi_hand_can(int idle_timeout_ms = 5, int max_wait_ms = 80);

void print_aoyi_hand_can_discovery(const vector<AoyiHandCanMatch> &matches);


vector<uint8_t> all_receive_aoyi_response(aoyi_hand side, int *right, int *left);

vector<uint8_t> receive_aoyi_response_quiet(aoyi_hand side, int *right, int *left);

int tiger_can_hand(aoyi_hand side);



int grasp_can_hand(aoyi_hand side);

int release_can_hand(aoyi_hand side);


// 返回 -1: 未抓物(6路状态全为0x02)或通信失败; 返回 0: 已抓物(有0x05等非0x02状态，会打印)
int get_can_hand_status(aoyi_hand side);

#endif
