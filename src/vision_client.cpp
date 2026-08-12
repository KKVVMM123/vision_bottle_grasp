#include "vision_client.h"

#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

int vision_connect(const std::string &host, int port, int max_retries, int retry_delay_ms)
{
    int attempts = 0;
    while (max_retries < 0 || attempts < max_retries)
    {
        ++attempts;
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
        {
            std::cerr << "[vision] socket 创建失败: " << strerror(errno) << std::endl;
            return -1;
        }

        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0)
        {
            std::cerr << "[vision] 无效地址: " << host << std::endl;
            close(sock);
            return -1;
        }

        if (connect(sock, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) == 0)
        {
            std::cout << "[vision] 已连接视觉服务器 " << host << ":" << port << std::endl;
            return sock;
        }

        close(sock);
        if (max_retries < 0 || attempts < max_retries)
        {
            std::cerr << "[vision] 连接失败(" << attempts << ")，" << retry_delay_ms << "ms 后重试" << std::endl;
            usleep(static_cast<useconds_t>(retry_delay_ms) * 1000);
        }
    }
    return -1;
}

bool vision_send_json(int fd, const std::string &json_str)
{
    if (fd < 0)
        return false;
    std::string payload = json_str + "\n";
    ssize_t sent = send(fd, payload.data(), payload.size(), 0);
    if (sent < 0)
    {
        std::cerr << "[vision] 发送失败: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

int vision_recv_line(int fd, std::string &line, double timeout_sec)
{
    if (fd < 0)
        return -1;

    line.clear();
    char buf[4096];
    std::string buffer;

    const int timeout_ms = static_cast<int>(timeout_sec * 1000.0);
    while (true)
    {
        // 检查 buffer 中是否已有完整一行
        size_t pos = buffer.find('\n');
        if (pos != std::string::npos)
        {
            line = buffer.substr(0, pos);
            return 1;
        }

        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        int pr = poll(&pfd, 1, timeout_ms);
        if (pr < 0)
        {
            if (errno == EINTR)
                continue;
            std::cerr << "[vision] poll 失败: " << strerror(errno) << std::endl;
            return -1;
        }
        if (pr == 0)
        {
            // 超时：连接仍有效，返回 0（调用方不要断开）
            if (!buffer.empty())
            {
                line = buffer;
                return 0; // 有部分数据但无换行，按超时处理
            }
            return 0;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
        {
            return -1;
        }

        ssize_t n = recv(fd, buf, sizeof(buf) - 1, 0);
        if (n > 0)
        {
            buf[n] = '\0';
            buffer.append(buf, static_cast<size_t>(n));
        }
        else if (n == 0)
        {
            return -1; // 对端关闭
        }
        else
        {
            if (errno == EINTR)
                continue;
            std::cerr << "[vision] recv 失败: " << strerror(errno) << std::endl;
            return -1;
        }
    }
}
