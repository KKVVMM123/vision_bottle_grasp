#ifndef BOTTLE_GRASP_VISION_CLIENT_H
#define BOTTLE_GRASP_VISION_CLIENT_H

#include <string>

// 连接视觉服务器（重试 max_retries 次，每次间隔 retry_delay_ms；-1 表示无限重试）
// 返回 socket fd，失败返回 -1
int vision_connect(const std::string &host, int port, int max_retries = -1, int retry_delay_ms = 1000);

// 发送一行 JSON（自动追加 '\n'），成功返回 true
bool vision_send_json(int fd, const std::string &json_str);

// 接收一行（直到 '\n' 或超时），timeout_sec 秒
// 返回 1=收到完整行；0=超时（连接仍有效，勿断开）；-1=断开/出错（需重连）
int vision_recv_line(int fd, std::string &line, double timeout_sec = 5.0);

#endif // BOTTLE_GRASP_VISION_CLIENT_H
