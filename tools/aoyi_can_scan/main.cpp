// 扫描 CAN 通道/HandID，或按配置测试灵巧手角度下发

#include "Ti5_socketcan.h"
#include "aoyihand.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

namespace
{

constexpr uint8_t kHeader1 = 0x55;
constexpr uint8_t kHeader2 = 0xAA;
constexpr uint8_t kMasterId = 0x01;

constexpr uint8_t kCmdGetProtocolVersion = 0x00;
constexpr uint8_t kCmdGetVendorId = 0x3F;

uint8_t calculateLRC(const uint8_t * data, size_t length)
{
  uint8_t lrc = 0;
  for (size_t i = 0; i < length; ++i) {
    lrc ^= data[i];
  }
  return lrc;
}

std::vector<uint8_t> buildReadCommand(uint8_t hand_id, uint8_t command)
{
  std::vector<uint8_t> packet = {
    kHeader1, kHeader2, hand_id, kMasterId, command, 0x00};
  const uint8_t checksum = calculateLRC(packet.data() + 2, 4);
  packet.push_back(checksum);
  return packet;
}

bool sendProtocolPacket(int channel, uint32_t can_id, const std::vector<uint8_t> & packet)
{
  size_t sent = 0;
  while (sent < packet.size()) {
    uint8_t frame[8] = {0};
    uint8_t dlc = 0;
    for (; dlc < 8 && sent < packet.size(); ++dlc, ++sent) {
      frame[dlc] = packet[sent];
    }
    if (!send_can_frame(channel, can_id, frame, dlc)) {
      return false;
    }
    usleep(1000);
  }
  return true;
}

void drainCanChannel(int channel, int rounds = 20)
{
  uint8_t dummy[8];
  for (int i = 0; i < rounds; ++i) {
    if (!receive_can_frame_timeout(channel, dummy)) {
      break;
    }
  }
}

bool isLikelyOHandResponse(const std::vector<uint8_t> & response, uint8_t hand_id, uint8_t command)
{
  if (response.size() < 7) {
    return false;
  }
  if (response[0] != kHeader1 || response[1] != kHeader2) {
    return false;
  }

  const uint8_t resp_hand_id = response[3];
  const uint8_t resp_cmd = response[4] & 0x7F;
  if (resp_hand_id != hand_id) {
    return false;
  }
  if (resp_cmd != command) {
    return false;
  }

  if (response.size() >= 6) {
    const uint8_t data_len = response[5];
    const size_t expected = 6 + data_len + 1;
    if (response.size() < expected) {
      return false;
    }
    const uint8_t checksum = response[expected - 1];
    const uint8_t calc = calculateLRC(response.data() + 2, 4 + data_len);
    if (checksum != calc) {
      return false;
    }
  }

  return true;
}

std::vector<uint8_t> receiveForMs(int channel, int timeout_ms)
{
  std::vector<uint8_t> full;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    uint8_t frame[8] = {0};
    if (receive_can_frame_timeout(channel, frame)) {
      for (int i = 0; i < 8; ++i) {
        full.push_back(frame[i]);
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  return full;
}

std::string toHex(const std::vector<uint8_t> & data)
{
  std::ostringstream oss;
  for (size_t i = 0; i < data.size(); ++i) {
    if (i > 0) {
      oss << ' ';
    }
    oss << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
        << static_cast<int>(data[i]);
  }
  return oss.str();
}

std::string describeVendor(const std::vector<uint8_t> & response)
{
  if (response.size() >= 8 && response[6] == 'O' && response[7] == 'Y') {
    return "vendor=OY";
  }
  if (response.size() >= 8) {
    return "data=" + toHex(std::vector<uint8_t>(response.begin() + 6, response.begin() + 8));
  }
  return "";
}

struct ProbeResult
{
  int channel = -1;
  int hand_id = -1;
  uint8_t command = 0;
  bool ok = false;
  std::vector<uint8_t> response;
};

ProbeResult probeOnce(int channel, int hand_id, uint8_t command, int timeout_ms)
{
  ProbeResult result;
  result.channel = channel;
  result.hand_id = hand_id;
  result.command = command;

  if (can_sockets[channel] < 0) {
    return result;
  }

  drainCanChannel(channel);

  const auto packet = buildReadCommand(static_cast<uint8_t>(hand_id), command);
  if (!sendProtocolPacket(channel, static_cast<uint32_t>(hand_id), packet)) {
    return result;
  }

  result.response = receiveForMs(channel, timeout_ms);
  result.ok = isLikelyOHandResponse(result.response, static_cast<uint8_t>(hand_id), command);
  return result;
}

const char * commandName(uint8_t cmd)
{
  switch (cmd) {
    case kCmdGetProtocolVersion:
      return "GET_PROTOCOL_VERSION(0x00)";
    case kCmdGetVendorId:
      return "GET_VENDOR_ID(0x3F)";
    default:
      return "UNKNOWN";
  }
}

void printResult(const ProbeResult & result)
{
  std::cout << "  channel=" << result.channel
            << "  hand_id=" << result.hand_id
            << "  cmd=" << commandName(result.command);

  if (result.ok) {
    std::cout << "  => 有回应";
    const auto extra = describeVendor(result.response);
    if (!extra.empty()) {
      std::cout << "  (" << extra << ")";
    }
    std::cout << "\n     HEX: " << toHex(result.response) << '\n';
  } else if (!result.response.empty()) {
    std::cout << "  => 收到数据但校验/格式不匹配\n"
              << "     HEX: " << toHex(result.response) << '\n';
  } else {
    std::cout << "  => 无回应\n";
  }
}

void printResponse(const std::vector<uint8_t> & response, const char * label)
{
  if (response.empty()) {
    std::cout << label << ": 无回应\n";
    return;
  }
  std::cout << label << " HEX: ";
  for (size_t i = 0; i < response.size(); ++i) {
    printf("%02X ", response[i]);
  }
  printf("\n");
}

int runCanScan()
{
  std::cout << "====== 傲意灵巧手 CAN 通道/ID 扫描 ======\n";
  std::cout << "协议: OHand V3，CAN ID = HandID，1Mbps\n";
  std::cout << "扫描通道: 0~5，HandID: 2, 3, 60, 70\n";
  std::cout << "查询命令: 0x3F(供应商ID) / 0x00(协议版本)\n\n";

  if (!init_socketcan()) {
    std::cerr << "init_socketcan 失败，请检查 can0~can5 是否存在\n";
    return 1;
  }

   int hand_ids[100] = {2, 3, 60, 70};
  for(int i=0;i<100;i++)
  {
    hand_ids[i]=i+1;
  }
  const uint8_t commands[] = {kCmdGetVendorId, kCmdGetProtocolVersion};
  std::vector<ProbeResult> hits;

  for (int channel = 0; channel < 8; ++channel) {
    if (can_sockets[channel] < 0) {
      std::cout << "[can" << channel << "] 未初始化，跳过\n\n";
      continue;
    }

    std::cout << "---- can" << channel << " ----\n";

    for (int hand_id : hand_ids) {
      for (uint8_t cmd : commands) {
        const auto result = probeOnce(channel, hand_id, cmd, 120);
        printResult(result);
        if (result.ok) {
          hits.push_back(result);
        }
        usleep(2000);
      }
    }
    std::cout << '\n';
  }

  std::cout << "====== 扫描结果汇总 ======\n";
  if (hits.empty()) {
    std::cout << "未发现有效回应。请检查:\n"
              << "  1. 灵巧手是否上电、CAN 线是否接对\n"
              << "  2. HandID 是否不在 {2,3,60,70} 中\n"
              << "  3. 通道号 right_can_f[0]/left_can_f[0] 是否在 0~5 内\n";
  } else {
    std::cout << "发现 " << hits.size() << " 组有效组合:\n";
    for (const auto & hit : hits) {
      std::cout << "  right/left_can_f = {" << hit.channel << ", " << hit.hand_id << "}"
                << "  (" << commandName(hit.command) << ")\n";
    }
    std::cout << "\n配置示例:\n";
    if (hits.size() >= 1) {
      std::cout << "  int right_can_f[2] = {" << hits[0].channel << ", " << hits[0].hand_id << "};\n";
    }
    if (hits.size() >= 2) {
      std::cout << "  int left_can_f[2]  = {" << hits[1].channel << ", " << hits[1].hand_id << "};\n";
    } else if (hits.size() == 1) {
      std::cout << "  int left_can_f[2]  = {" << hits[0].channel << ", " << hits[0].hand_id << "};\n";
    }
  }

  close_can_channels();
  return hits.empty() ? 2 : 0;
}

int runHandTest()
{
  std::cout << "====== 灵巧手角度测试 ======\n";

  if (!init_socketcan()) {
    std::cerr << "init_socketcan 失败，请检查 can0~can5 是否存在\n";
    return 1;
  }

  int right_can_f[2] = {4, 70};
  int left_can_f[2] = {5, 60};
  uint16_t final_angles_right[6] = {0, 90, 90, 0, 90, 0};
  uint16_t final_angles_left[6] = {0, 90, 90, 0, 90, 0};
  uint8_t speeds[6] = {255, 255, 255, 255, 255, 255};

  std::cout << "right_can_f={" << right_can_f[0] << ", " << right_can_f[1] << "}\n";
  std::cout << "left_can_f={" << left_can_f[0] << ", " << left_can_f[1] << "}\n";

  aoyijiaodu_via_can(final_angles_right, speeds, right_a, right_can_f, left_can_f);
 

  aoyijiaodu_via_can(final_angles_left, speeds, left_a, right_can_f, left_can_f);
 

  close_can_channels();
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc >= 2 && std::strcmp(argv[1], "scan") == 0) {
    return runCanScan();
  }
  if (argc >= 2 && std::strcmp(argv[1], "test") == 0) {
    return runHandTest();
  }

  std::cout << "用法:\n"
            << "  sudo ./aoyi_can_scan scan   # 扫描通道/HandID\n"
            << "  sudo ./aoyi_can_scan test   # 按 right/left_can_f 测试角度\n";
  return 1;
}
