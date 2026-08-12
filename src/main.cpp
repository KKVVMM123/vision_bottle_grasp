#include "motion.h"
#include "vision_client.h"
#include "aoyihand.h"

#include <nlohmann/json.hpp>

#include <csignal>
#include <atomic>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <thread>
#include <chrono>

using json = nlohmann::json;

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_home_requested{false};

// 终端非阻塞读（供键盘快捷键）
static bool setStdinNonBlocking(bool enable)
{
    static struct termios oldt, newt;
    if (enable)
    {
        if (!isatty(STDIN_FILENO))
            return false;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    else
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }
    return true;
}

static void onSignal(int)
{
    g_running = false;
}

// 读取头部角并组装 CLIENT_READY JSON
static std::string buildReadyJson()
{
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    readHeadAnglesDeg(roll, pitch, yaw);
    std::cout << "[main] 头部角(deg) roll=" << roll << " pitch=" << pitch << " yaw=" << yaw << std::endl;
    json j;
    j["type"] = "CLIENT_READY";
    j["head_roll"] = roll;
    j["head_pitch"] = pitch;
    j["head_yaw"] = yaw;
    return j.dump();
}

int main(int argc, char **argv)
{
    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);

    GraspConfig cfg;
    loadGraspConfig("config/motion.txt", cfg);

    std::cout << "=== bottle_grasp C++ 主控 ===" << std::endl;
    std::cout << "TCP " << cfg.tcp_host << ":" << cfg.tcp_port
              << "  dry_run=" << (cfg.dry_run ? "true" : "false") << std::endl;

    // 初始化 CAN（需要 root：sudo ./build/bottle_grasp）
    if (!init_socketcan())
    {
        std::cerr << "CAN 初始化失败，请确认以 root 运行且 can0~can7 存在" << std::endl;
        return 1;
    }
    hand_socketid_bind();
    std::cout << "CAN 初始化完成 (head_can_id=" << head_can_id << ")" << std::endl;

    // 右臂与手
    Robot arm(-1);
    handOpen(cfg);
    moveRightToReady(arm);

    // 键盘快捷键线程：h=回预备位，q=退出
    bool stdin_tty = setStdinNonBlocking(true);
    std::thread kb_thread([&]()
                          {
                              if (!stdin_tty)
                                  return;
                              char c;
                              while (g_running)
                              {
                                  ssize_t n = read(STDIN_FILENO, &c, 1);
                                  if (n == 1)
                                  {
                                      if (c == 'h' || c == 'H')
                                      {
                                          std::cout << "[main] 键盘指令: 回预备位" << std::endl;
                                          g_home_requested = true;
                                      }
                                      else if (c == 'q' || c == 'Q')
                                      {
                                          std::cout << "[main] 键盘指令: 退出" << std::endl;
                                          g_running = false;
                                      }
                                  }
                                  usleep(50000);
                              }
                          });

    int sock = -1;
    bool holding = false;
    auto last_head_push = std::chrono::steady_clock::now();

    while (g_running)
    {
        // 周期主动推送头部角给视觉（视觉始终用最新值，无需请求）
        {
            auto now_t = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_t - last_head_push).count();
            if (elapsed >= cfg.head_push_interval_ms)
            {
                last_head_push = now_t;
                if (sock >= 0)
                {
                    double hr = 0, hp = 0, hy = 0;
                    readHeadAnglesDeg(hr, hp, hy);
                    json j;
                    j["type"] = "HEAD_ANGLES";
                    j["head_roll"] = hr;
                    j["head_pitch"] = hp;
                    j["head_yaw"] = hy;
                    vision_send_json(sock, j.dump());
                }
            }
        }

        if (g_home_requested.exchange(false))
        {
            std::cout << "[main] 执行回预备位..." << std::endl;
            releaseHand(arm, cfg);
            holding = false;
            if (sock >= 0)
            {
                json resp;
                resp["type"] = "HOME_DONE";
                resp["result"] = "ok";
                vision_send_json(sock, resp.dump());
            }
        }

        if (sock < 0)
        {
            std::cout << "等待连接视觉服务器 " << cfg.tcp_host << ":" << cfg.tcp_port << " ..." << std::endl;
            sock = vision_connect(cfg.tcp_host, cfg.tcp_port, 1, 1000);
            if (sock < 0)
            {
                sleep(1);
                continue;
            }
            vision_send_json(sock, buildReadyJson());
            std::cout << "已连接，上报头部角完成" << std::endl;
        }

        std::string line;
        int rv = vision_recv_line(sock, line, 0.2);
        if (rv < 0)
        {
            if (!g_running)
                break;
            close(sock);
            sock = -1;
            std::cout << "视觉连接断开，准备重连..." << std::endl;
            continue;
        }
        if (rv == 0)
        {
            // 超时无消息：连接仍有效，保持不重连
            continue;
        }

        try
        {
            json msg = json::parse(line);
            std::string type = msg.value("type", "");

            if (type == "GRASP")
            {
                double goal[3] = {msg.value("x", 0.0), msg.value("y", 0.0), msg.value("z", 0.0)};
                double yaw_deg = msg.value("yaw_deg", cfg.grasp_yaw_deg);
                std::string cls = msg.value("class_name", "");
                std::cout << "[main] 收到抓取指令: " << cls
                          << " xyz=(" << goal[0] << "," << goal[1] << "," << goal[2]
                          << ") yaw=" << yaw_deg << std::endl;

                // 若上一轮还在保持，先释放并回预备位
                if (holding)
                {
                    std::cout << "[main] 先释放上一轮抓取..." << std::endl;
                    releaseHand(arm, cfg);
                    holding = false;
                }

                double depth_override = msg.value("depth_offset_m", 0.0);
                int ret = executeGrasp(arm, goal, yaw_deg, cfg, depth_override);
                holding = (ret == 0);

                json resp;
                resp["type"] = "GRASP_DONE";
                if (ret == 0)
                    resp["result"] = "ok";
                else if (ret == 1)
                    resp["result"] = "not_grasped";
                else
                    resp["result"] = "motion_error";
                resp["holding"] = holding;
                resp["ret"] = ret;
                {
                    double hr, hp, hy;
                    readHeadAnglesDeg(hr, hp, hy);
                    resp["head_roll"] = hr;
                    resp["head_pitch"] = hp;
                    resp["head_yaw"] = hy;
                }
                vision_send_json(sock, resp.dump());
            }
            else if (type == "GET_HEAD_ANGLES")
            {
                vision_send_json(sock, buildReadyJson());
            }
            else if (type == "HOME" || type == "RELEASE")
            {
                releaseHand(arm, cfg);
                holding = false;
                json resp;
                resp["type"] = (type == "HOME") ? "HOME_DONE" : "RELEASE_DONE";
                resp["result"] = "ok";
                vision_send_json(sock, resp.dump());
            }
            else if (type == "STATUS")
            {
                json resp;
                resp["type"] = "STATUS";
                resp["holding"] = holding;
                resp["head_can_id"] = head_can_id;
                {
                    double hr, hp, hy;
                    readHeadAnglesDeg(hr, hp, hy);
                    resp["head_roll"] = hr;
                    resp["head_pitch"] = hp;
                    resp["head_yaw"] = hy;
                }
                vision_send_json(sock, resp.dump());
            }
            else if (type == "QUIT")
            {
                std::cout << "[main] 收到 QUIT，退出" << std::endl;
                break;
            }
            else
            {
                std::cout << "[main] 未知消息: " << line << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "[main] JSON 解析错误: " << e.what() << " line=" << line << std::endl;
        }
    }

    // 退出清理：张开手 + 关闭 CAN + 恢复终端
    g_running = false;
    if (kb_thread.joinable())
        kb_thread.join();
    if (stdin_tty)
        setStdinNonBlocking(false);
    handOpen(cfg);
    if (sock >= 0)
        close(sock);
    close_can_channels();
    std::cout << "[main] 已退出" << std::endl;
    return 0;
}
