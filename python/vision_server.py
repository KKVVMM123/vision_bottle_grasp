#!/usr/bin/env python3
"""
bottle_grasp 视觉服务
====================
RealSense + YOLO-Seg -> 实例掩码 -> 深度点云重心 -> 头部角+标定 -> 机器人 base 系坐标
OpenCV 窗口：点击检测到的瓶子 -> 通过 TCP 通知 C++ 主控执行抓取。

用法：
  python3 vision_server.py                          # 在线模式（RealSense + TCP + GUI）
  python3 vision_server.py --offline RGB.png DEPTH.png [--show]
                                                    # 离线测试：用真实图片，不连 TCP
  python3 vision_server.py --offline RGB.png --mock # 离线 + 模拟掩码（无 YOLO 依赖）

键盘：q 退出；r 重新向 C++ 同步头部角。
"""

from __future__ import annotations

import argparse
import json
import select
import socket
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from head_camera_to_base import HeadCameraToBase

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = Path(__file__).resolve().parent / "config" / "vision.yaml"


def load_config(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


class VisionServer:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.cam_cfg = cfg["camera"]
        self.model_cfg = cfg["model"]
        self.head_cfg = cfg["head"]
        self.tcp_cfg = cfg["tcp"]

        # 相机 -> 机器人 base 变换（yaml 路径相对本文件所在目录 python/ 解析）
        calib_path = (Path(__file__).resolve().parent / self.cfg["calibration"]["yaml"]).resolve()
        self.headkin = HeadCameraToBase.from_yaml(
            calib_path, float(self.cfg["calibration"]["link_length_mm"])
        )
        print(f"[vision] 标定: {calib_path}  link_length_mm={self.cfg['calibration']['link_length_mm']}")

        # TCP 服务端（C++ 为客户端）；仅在线模式创建
        self.server = None
        self.client = None

        # 头部角缓存（初始化为配置默认；成功读到真实值后持续更新，失败时用缓存而非默认）
        self._head_angles = (
            float(self.head_cfg["default_roll_deg"]),
            float(self.head_cfg["default_pitch_deg"]),
            float(self.head_cfg["default_yaw_deg"]),
        )
        self._head_angles_time = 0.0
        self._angles_lock = threading.Lock()

        # GUI 状态
        self._click_pos = None
        self._click_flag = False

        # 模型（--mock 时不加载）
        self.model = None
        self.class_names = []
        self.mock = False

    def _init_tcp_server(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.tcp_cfg["host"], self.tcp_cfg["port"]))
        self.server.listen(1)
        self.server.setblocking(False)
        print(f"[vision] TCP 服务监听 {self.tcp_cfg['host']}:{self.tcp_cfg['port']}")

    # ---------------- 模型 ----------------
    def load_model(self):
        from ultralytics import YOLO  # 延迟导入，避免无依赖时无法启动

        self.model = YOLO(self.model_cfg["path"])
        self.class_names = self.model.names
        print(f"[vision] 模型加载完成: {self.model_cfg['path']} device={self.model_cfg['device']}")

    def predict(self, bgr: np.ndarray):
        """返回 list[dict]: {class_id, class_name, confidence, mask(0/255 uint8, 与图像同尺寸)}"""
        if self.mock:
            return self._mock_detections(bgr)
        if self.model is None:
            return []
        classes = self.model_cfg.get("classes") or None
        if classes:
            # 类别名 -> id
            id_map = {name: i for i, name in self.class_names.items()}
            classes = [id_map[c] for c in classes if c in id_map]
            classes = classes or None
        results = self.model.predict(bgr, conf=self.model_cfg["confidence"], classes=classes, verbose=False)
        dets = []
        if not results:
            return dets
        r = results[0]
        masks = getattr(r, "masks", None)
        if masks is None:
            return dets
        h, w = bgr.shape[:2]
        mask_data = masks.data.cpu().numpy()  # (N, H', W')
        for i in range(len(r.boxes)):
            cls_id = int(r.boxes.cls[i])
            conf = float(r.boxes.conf[i])
            mask = cv2.resize(mask_data[i], (w, h), interpolation=cv2.INTER_LINEAR)
            mask = (mask > 0.5).astype(np.uint8) * 255
            dets.append({
                "class_id": cls_id,
                "class_name": self.class_names.get(cls_id, str(cls_id)),
                "confidence": conf,
                "mask": mask,
            })
        return dets

    def _mock_detections(self, bgr: np.ndarray):
        h, w = bgr.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        center = (int(w * 0.5), int(h * 0.5))
        axes = (int(w * 0.12), int(h * 0.25))
        cv2.ellipse(mask, center, axes, 0, 0, 360, 255, -1)
        return [{
            "class_id": 0,
            "class_name": "mock_bottle",
            "confidence": 1.0,
            "mask": mask,
        }]

    # ---------------- 头部角 ----------------
    def _request_head_angles(self):
        if self.client is None:
            return None
        try:
            self.client.sendall(b'{"type":"GET_HEAD_ANGLES"}\n')
            self.client.settimeout(self.tcp_cfg["recv_timeout_sec"])
            data = self.client.recv(4096)
            if not data:
                return None
            for line in data.decode("utf-8", "replace").splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except Exception:
                    continue
                if "head_roll" in msg:
                    return (float(msg["head_roll"]), float(msg["head_pitch"]), float(msg["head_yaw"]))
            return None
        except Exception:
            return None

    def get_head_angles(self, force: bool = False):
        now = time.time()
        ttl = float(self.head_cfg.get("angles_ttl_sec", 1.0))
        with self._angles_lock:
            if (not force and self._head_angles is not None
                    and now - self._head_angles_time < ttl):
                return self._head_angles
        angles = self._request_head_angles()
        if angles is not None:
            with self._angles_lock:
                self._head_angles = angles
                self._head_angles_time = now
            return angles
        # 请求失败（如 C++ 正在抓取没空响应）：用最近一次有效头部角，不回退默认
        with self._angles_lock:
            if self._head_angles is not None and self._head_angles_time > 0:
                return self._head_angles
        return (
            float(self.head_cfg["default_roll_deg"]),
            float(self.head_cfg["default_pitch_deg"]),
            float(self.head_cfg["default_yaw_deg"]),
        )

    # ---------------- 三维 ----------------
    def instance_pose(self, mask: np.ndarray, depth_m: np.ndarray, intrinsics):
        """实例掩码 -> 点云重心 -> 相机系坐标 -> 机器人 base 系坐标"""
        if depth_m.ndim != 2:
            print(f"[vision] 深度图必须是单通道 2D（uint16 mm），当前 shape={depth_m.shape}")
            return None
        ys, xs = np.nonzero(mask > 0)
        if len(xs) < 100:
            return None
        d = depth_m[ys, xs].astype(np.float64)
        valid = (d >= float(self.cam_cfg["min_depth_m"])) & (d <= float(self.cam_cfg["max_depth_m"]))
        if valid.sum() < 100:
            return None
        xs_v, ys_v, d_v = xs[valid], ys[valid], d[valid]
        fx = float(intrinsics.fx)
        fy = float(intrinsics.fy)
        ppx = float(intrinsics.ppx)
        ppy = float(intrinsics.ppy)
        x = (xs_v - ppx) * d_v / fx
        y = (ys_v - ppy) * d_v / fy
        z = d_v
        centroid_cam = np.array([x.mean(), y.mean(), z.mean()])
        return centroid_cam

    def instance_cloud_robot(self, mask: np.ndarray, depth_m: np.ndarray, intrinsics):
        """掩码内有效深度点 -> 机器人 base 系点云 (N,3)。"""
        if depth_m.ndim != 2:
            return None
        ys, xs = np.nonzero(mask > 0)
        if len(xs) < 100:
            return None
        d = depth_m[ys, xs].astype(np.float64)
        valid = (d >= float(self.cam_cfg["min_depth_m"])) & (d <= float(self.cam_cfg["max_depth_m"]))
        if valid.sum() < 100:
            return None
        xs_v, ys_v, d_v = xs[valid], ys[valid], d[valid]
        fx = float(intrinsics.fx); fy = float(intrinsics.fy)
        ppx = float(intrinsics.ppx); ppy = float(intrinsics.ppy)
        cam = np.stack([(xs_v - ppx) * d_v / fx,
                        (ys_v - ppy) * d_v / fy,
                        d_v], axis=1)
        T = self.headkin.compute_cam2robot_meters(*self.get_head_angles())
        robot = (T[:3, :3] @ cam.T).T + T[:3, 3]
        return robot

    def cam_to_robot(self, centroid_cam: np.ndarray):
        roll, pitch, yaw = self.get_head_angles()
        T = self.headkin.compute_cam2robot_meters(roll, pitch, yaw)
        p_robot = T[:3, :3] @ centroid_cam + T[:3, 3]
        return p_robot, (roll, pitch, yaw)

    # ---------------- TCP 下发 ----------------
    def send_grasp(self, class_name: str, xyz: np.ndarray, yaw_deg: float, depth_offset_m: float = None):
        if self.client is None:
            print(f"[vision] C++ 未连接，无法下发抓取: {class_name} {xyz}")
            return False
        msg = {
            "type": "GRASP",
            "class_name": class_name,
            "x": round(float(xyz[0]), 6),
            "y": round(float(xyz[1]), 6),
            "z": round(float(xyz[2]), 6),
            "yaw_deg": float(yaw_deg),
        }
        if depth_offset_m is not None:
            msg["depth_offset_m"] = round(float(depth_offset_m), 6)
        try:
            self.client.sendall((json.dumps(msg) + "\n").encode("utf-8"))
            print(f"[vision] 已下发抓取: {msg}")
            return True
        except Exception as e:
            print(f"[vision] 下发失败: {e}")
            self.client = None
            return False

    # ---------------- 诊断 ----------------
    def _dump_detections(self, color: np.ndarray, overlay: np.ndarray):
        """按 's' 触发：低阈值跑一次检测并打印全部结果，保存截图到 logs/snapshots/。"""
        import time as _t
        self._last_depth_m = getattr(self, "_last_depth_m", None)
        out_dir = PROJECT_ROOT / "logs" / "snapshots"
        out_dir.mkdir(parents=True, exist_ok=True)
        ts = _t.strftime("%Y%m%d_%H%M%S")
        print(f"\n[vision] === 检测诊断 {ts} ===")
        if self.model is None:
            print("[vision] 模型未加载")
            return
        # 用当前头部角计算每个实例的机器人系坐标（供核对）
        roll, pitch, yaw = self.get_head_angles(force=True)
        print(f"[vision] 当前头部角(deg): roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f}")
        for conf_thr in (0.35, 0.15, 0.05):
            res = self.model.predict(color, conf=conf_thr, verbose=False)[0]
            n = 0 if res.boxes is None else len(res.boxes)
            print(f"[vision] conf>={conf_thr}: {n} 个目标")
            if n:
                masks = getattr(res, "masks", None)
                for i in range(n):
                    cls = int(res.boxes.cls[i]); cf = float(res.boxes.conf[i])
                    xy = [round(float(v), 0) for v in res.boxes.xyxy[i]]
                    line = f"    {self.class_names.get(cls)} conf={cf:.3f} box={xy}"
                    if masks is not None and self.client is not None:
                        m = cv2.resize(masks.data[i].cpu().numpy(), (color.shape[1], color.shape[0]),
                                       interpolation=cv2.INTER_LINEAR)
                        m = (m > 0.5).astype(np.uint8) * 255
                        intr = _IntrinsicsLike(color.shape[1], color.shape[0])
                        cc = self.instance_pose(m, self._last_depth_m, intr)
                        if cc is not None:
                            pr, _ = self.cam_to_robot(cc)
                            line += f"  robot=({pr[0]:.3f},{pr[1]:.3f},{pr[2]:.3f})"
                    print(line)
        raw_path = out_dir / f"raw_{ts}.jpg"
        ann_path = out_dir / f"annotated_{ts}.jpg"
        cv2.imwrite(str(raw_path), color)
        cv2.imwrite(str(ann_path), overlay)
        print(f"[vision] 已保存: {raw_path} / {ann_path}")

    # ---------------- 事件处理 ----------------
    def _accept_client(self):
        if self.server is None:
            return
        try:
            conn, addr = self.server.accept()
            conn.setblocking(True)
            if self.client is not None:
                try:
                    self.client.close()
                except Exception:
                    pass
            self.client = conn
            print(f"[vision] C++ 客户端已连接: {addr}")
            self._head_angles = None  # 强制重新同步头部角
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"[vision] accept 错误: {e}")

    def _drain_client(self):
        """非阻塞读取 C++ 发回的消息（GRASP_DONE/STATUS/...），避免残留在缓冲区。"""
        if self.client is None:
            return
        self.client.setblocking(False)
        try:
            while True:
                data = self.client.recv(4096)
                if not data:
                    print("[vision] C++ 客户端断开")
                    try:
                        self.client.close()
                    except Exception:
                        pass
                    self.client = None
                    return
                for line in data.decode("utf-8", "replace").splitlines():
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                        t = msg.get("type", "?")
                        if "head_roll" in msg:
                            with self._angles_lock:
                                self._head_angles = (float(msg["head_roll"]),
                                                     float(msg["head_pitch"]),
                                                     float(msg["head_yaw"]))
                                self._head_angles_time = time.time()
                        if t == "HEAD_ANGLES":
                            continue  # 周期推送，静默更新缓存
                        if t == "GRASP_DONE":
                            print(f"[vision] 抓取完成: result={msg.get('result')} holding={msg.get('holding')}")
                        else:
                            print(f"[vision] 收到 C++: {line}")
                    except Exception:
                        print(f"[vision] 收到 C++ 原始: {line}")
        except BlockingIOError:
            pass  # 无更多数据
        except Exception as e:
            print(f"[vision] 读取 C++ 消息出错: {e}")
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None
        finally:
            if self.client is not None:
                try:
                    self.client.setblocking(True)
                except Exception:
                    pass

    def _on_click(self, dets, depth_m, intrinsics):
        x, y = self._click_pos
        for det in dets:
            if det["mask"][y, x] > 0:
                cloud = self.instance_cloud_robot(det["mask"], depth_m, intrinsics)
                if cloud is None:
                    print(f"[vision] 目标 '{det['class_name']}' 深度无效，无法计算位姿")
                    return
                centroid_robot = cloud.mean(axis=0)
                # 工作空间预检（与 C++ 一致）：法兰到 p2 距离 >0.522 就不下发
                flange = float(np.sqrt((centroid_robot[0] - 0.15) ** 2 +
                                       (centroid_robot[1] + 0.192) ** 2 +
                                       centroid_robot[2] ** 2))
                if flange > 0.52:
                    print(f"[vision] 警告：瓶子太远！法兰距离 {flange:.2f}>0.52，"
                          f"请把瓶子放到机器人前方 0.4~0.5m（画面里 x 显示绿色），本次不下发抓取")
                    return
                # 前表面：点云 x 的 5% 分位（机器人从 -x 侧接近，前表面=最小 x）
                front_x = float(np.percentile(cloud[:, 0], 5))
                penetration = float(self.cfg["grasp"].get("front_penetration_m", 0.01))
                # 深度补偿 = 前表面 + 小穿透 - 重心x（C++ 抓取点 = x + depth_offset）
                depth_offset_m = front_x - float(centroid_robot[0]) + penetration
                angles = self.get_head_angles()
                yaw = float(self.cfg["grasp"]["yaw_deg"])
                print(f"[vision] 选中 {det['class_name']} conf={det['confidence']:.2f}")
                print(f"[vision] 头部角(deg)={angles}")
                print(f"[vision] 机器人系重心(m)={np.round(centroid_robot, 4)}")
                print(f"[vision] 前表面 x={front_x:.4f} 穿透 {penetration:.4f} -> 深度补偿 {depth_offset_m:.4f}m")
                self.send_grasp(det["class_name"], centroid_robot, yaw, depth_offset_m)
                return
        print("[vision] 点击处没有检测到目标")

    # ---------------- 在线主循环 ----------------
    def _start_camera_with_retry(self, rs, pipeline, config, max_tries=2):
        """启动相机并等待首帧；失败则 hardware_reset 该设备后重试。
        解决相机被异常停止后卡死、不出帧（Frame didn't arrive）的问题。"""
        serial = self.cam_cfg.get("serial")
        for attempt in range(1, max_tries + 1):
            pipeline.start(config)
            try:
                fs = pipeline.wait_for_frames(timeout_ms=8000)
                if fs.get_color_frame() is not None:
                    print(f"[vision] 相机出帧正常 (attempt {attempt})")
                    return True
            except Exception:
                pass
            print(f"[vision] 相机 {serial or '(auto)'} 等待首帧失败 (attempt {attempt})，准备复位重试...")
            try:
                pipeline.stop()
            except Exception:
                pass
            if serial:
                try:
                    for dev in rs.context().devices:
                        if dev.get_info(rs.camera_info.serial_number) == serial:
                            dev.hardware_reset()
                            print(f"[vision] 已复位设备 {serial}")
                            break
                except Exception as e:
                    print(f"[vision] 复位失败: {e}")
                time.sleep(8)
        return False

    def run_online(self):
        import pyrealsense2 as rs  # 延迟导入

        self._init_tcp_server()
        # 关键：在线模式也必须加载检测模型（之前漏了，导致永远检测不到目标）
        if not self.mock:
            try:
                self.load_model()
            except Exception as e:
                print(f"[vision] 模型加载失败: {e}，请检查 model.path/device")
                self.server.close()
                return 1
        pipeline = rs.pipeline()
        config = rs.config()
        if self.cam_cfg.get("serial"):
            config.enable_device(self.cam_cfg["serial"])
        config.enable_stream(rs.stream.color, self.cam_cfg["width"], self.cam_cfg["height"],
                             rs.format.bgr8, self.cam_cfg["fps"])
        config.enable_stream(rs.stream.depth, self.cam_cfg["width"], self.cam_cfg["height"],
                             rs.format.z16, self.cam_cfg["fps"])
        if not self._start_camera_with_retry(rs, pipeline, config):
            print("[vision] 相机多次尝试仍无法出帧，退出")
            self.server.close()
            return 1

        # 对齐到彩色
        align = rs.align(rs.stream.color)

        window = "bottle_grasp"
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

        def mouse_cb(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                self._click_pos = (x, y)
                self._click_flag = True

        cv2.setMouseCallback(window, mouse_cb)

        print("[vision] 在线模式运行中（q 退出，r 重同步头部角，点击目标下发抓取）")
        try:
            while True:
                self._accept_client()
                self._drain_client()

                frames = pipeline.wait_for_frames(timeout_ms=5000)
                aligned = align.process(frames)
                color = np.asanyarray(aligned.get_color_frame().get_data())
                depth = np.asanyarray(aligned.get_depth_frame().get_data())
                depth_m = depth.astype(np.float64) * float(self.cam_cfg["depth_scale"])
                self._last_depth_m = depth_m

                intrinsics = aligned.get_color_frame().profile.as_video_stream_profile().intrinsics

                dets = self.predict(color)

                overlay = color.copy()
                for det in dets:
                    cnts, _ = cv2.findContours(det["mask"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(overlay, cnts, -1, (0, 200, 0), -1)
                overlay = cv2.addWeighted(color, 0.6, overlay, 0.4, 0)
                for det in dets:
                    ys, xs = np.nonzero(det["mask"] > 0)
                    if len(xs) == 0:
                        continue
                    x0, y0, x1, y1 = xs.min(), ys.min(), xs.max(), ys.max()
                    cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 255, 0), 2)
                    label = f"{det['class_name']} {det['confidence']:.2f}"
                    cv2.putText(overlay, label, (x0, max(0, y0 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    # 实时显示机器人系距离与法兰距离（超出工作空间显示红色）
                    cc = self.instance_pose(det["mask"], depth_m, intrinsics)
                    if cc is not None:
                        pr, _ = self.cam_to_robot(cc)
                        flange = float(np.sqrt((pr[0] - 0.15) ** 2 +
                                               (pr[1] + 0.192) ** 2 +
                                               pr[2] ** 2))
                        ok = flange <= 0.52
                        dcol = (0, 255, 0) if ok else (0, 0, 255)
                        cv2.putText(overlay, f"x={pr[0]:.2f} 法兰={flange:.2f}",
                                    (x0, y1 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    dcol, 2)

                cv2.putText(overlay, f"detections: {len(dets)}", (20, 72),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                if self.client is None:
                    cv2.putText(overlay, "NO C++ CLIENT", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                else:
                    cv2.putText(overlay, "C++ CLIENT OK", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

                if self._click_flag:
                    self._click_flag = False
                    self._on_click(dets, depth_m, intrinsics)

                cv2.imshow(window, overlay)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("r"):
                    self.get_head_angles(force=True)
                    print("[vision] 已重新同步头部角")
                elif key == ord("h"):
                    if self.client is not None:
                        try:
                            self.client.sendall(b'{"type":"HOME"}\n')
                            print("[vision] 已发送回预备位指令 (HOME)")
                        except Exception as e:
                            print(f"[vision] 发送失败: {e}")
                    else:
                        print("[vision] C++ 未连接，无法回预备位")
                elif key == ord("s"):
                    self._dump_detections(color, overlay)
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()
            self.server.close()
            if self.client is not None:
                self.client.close()

    # ---------------- 离线 ----------------
    def run_offline(self, rgb_path: Path, depth_path: Path | None, show: bool, mock: bool):
        self.mock = mock
        rgb = cv2.imread(str(rgb_path))
        if rgb is None:
            print(f"[vision] 无法读取 RGB 图片: {rgb_path}")
            return 1
        h, w = rgb.shape[:2]
        if depth_path is not None and depth_path.exists():
            depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
            if depth is None:
                print(f"[vision] 无法读取深度图: {depth_path}")
                return 1
        else:
            print("[vision] 未提供深度图，使用平面深度 0.5m 模拟")
            depth = np.full((h, w), int(0.5 / float(self.cam_cfg["depth_scale"])), dtype=np.uint16)
        depth_m = depth.astype(np.float64) * float(self.cam_cfg["depth_scale"])

        # 相机内参：离线模式下使用配置默认（或从图片尺寸近似）
        intrinsics = _default_intrinsics(w, h)
        if not self.mock:
            self.load_model()
        dets = self.predict(rgb)
        print(f"[vision] 检测到 {len(dets)} 个目标")

        for det in dets:
            centroid_cam = self.instance_pose(det["mask"], depth_m, intrinsics)
            if centroid_cam is None:
                print(f"[vision] {det['class_name']}: 深度无效")
                continue
            p_robot, angles = self.cam_to_robot(centroid_cam)
            print(f"[vision] {det['class_name']} conf={det['confidence']:.2f}")
            print(f"   头部角(deg)={angles}")
            print(f"   相机系重心(m)={np.round(centroid_cam, 4)}")
            print(f"   机器人系重心(m)={np.round(p_robot, 4)}")

        if show:
            overlay = rgb.copy()
            for det in dets:
                ys, xs = np.nonzero(det["mask"] > 0)
                if len(xs) == 0:
                    continue
                cv2.rectangle(overlay, (xs.min(), ys.min()), (xs.max(), ys.max()), (0, 255, 0), 2)
                cv2.putText(overlay, det["class_name"], (xs.min(), max(0, ys.min() - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("bottle_grasp offline", overlay)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return 0


class _IntrinsicsLike:
    """简易内参容器（无 pyrealsense 时用于诊断打印）。"""
    def __init__(self, width, height):
        self.fx = 911.3588
        self.fy = 910.6850
        self.ppx = width / 2.0
        self.ppy = height / 2.0
        self.width = width
        self.height = height


def _default_intrinsics(width: int, height: int):
    """离线模式无 RealSense 时构造的简易内参（仅用于几何验证）。"""
    class _Intrinsics:
        pass
    intr = _Intrinsics()
    intr.fx = 909.45
    intr.fy = 908.23
    intr.ppx = width / 2.0
    intr.ppy = height / 2.0
    intr.width = width
    intr.height = height
    return intr


def main():
    parser = argparse.ArgumentParser(description="bottle_grasp 视觉服务")
    parser.add_argument("--config", default=str(CONFIG_PATH), help="vision.yaml 路径")
    parser.add_argument("--offline", nargs="*", default=None,
                        help="离线模式: --offline RGB.png [DEPTH.png]")
    parser.add_argument("--mock", action="store_true", help="使用模拟掩码（不加载 YOLO）")
    parser.add_argument("--show", action="store_true", help="离线模式显示窗口")
    args = parser.parse_args()

    cfg = load_config(Path(args.config))
    server = VisionServer(cfg)

    if args.offline:
        if not args.offline:
            print("离线模式需要至少一个 RGB 图片路径")
            return 1
        rgb_path = Path(args.offline[0])
        depth_path = Path(args.offline[1]) if len(args.offline) > 1 else None
        return server.run_offline(rgb_path, depth_path, args.show, args.mock)

    server.mock = args.mock
    server.run_online()


if __name__ == "__main__":
    raise SystemExit(main())
