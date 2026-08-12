#ifndef TI5_SOCKETCAN
#define TI5_SOCKETCAN


#include <iostream>
#include <string>
#include <cstring>
#include <array>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <vector>  // 添加 vector 头文件
#include <thread>  

using namespace std;

const int CAN_CHANNEL_COUNT = 8;  // can0 ~ can13

extern int left_h_id,left_m_id,left_l_id,right_h_id,right_m_id,right_l_id,waist_can_id ,head_can_id ;

extern std::array<int, CAN_CHANNEL_COUNT> can_sockets;

typedef struct {
    uint32_t AccCode;
    uint32_t AccMask;
    uint8_t Filter;
    uint32_t Bitrate;
    uint8_t Mode;
} CAN_CONFIG;

bool set_can_bitrate(const std::string &ifname, uint32_t bitrate);

bool init_can_channel(int channel, const CAN_CONFIG &config);

bool init_socketcan();

void close_can_channels();

bool send_can_frame(int channel, uint32_t can_id, const uint8_t *data, uint8_t dlc);

bool receive_can_frame_timeout(int channel, uint8_t *data);

void toIntArray(int number, uint8_t *res, int size);

int socketcan_sendcommand(int channel, int cannum, uint32_t *can_idlist, uint8_t command, int *data);

int socketcan_sendsimplecommand(int channel, int cannum, uint32_t *can_idlist, uint8_t command, int *data);


bool select_bind(int &hand_id, int id, string name);

void hand_socketid_bind();

void get_motor_position(int *data, int a);

void set_motor_position(int *data, int a);

void get_motor_speed(int *data, int a);

void set_motor_speed(int *data, int a);


void set_motor_addspeed(int *data, int a);

void get_motor_addspeed(int *data, int a);

void set_motor(int *data, uint8_t command,int a);

void set_motor_singn_mode(int *data,uint8_t command, int a);

void set_motor_current_zero();


void set_waist_motor_position(int *data);



void get_motor_waist_position(int *data);

void motor_speed_change();





#endif