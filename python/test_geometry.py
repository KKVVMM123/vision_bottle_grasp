#!/usr/bin/env python3
"""纯 numpy 几何单测：验证 头部角+标定 -> cam2robot 变换 与 深度重心换算。

不需要 torch/ultralytics/pyrealsense2，可在任意机器运行：
  python3 test_geometry.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from head_camera_to_base import HeadCameraToBase, euler_to_rotation_matrix


def test_identity_transform():
    """零头部角 + 已知 T_head_cam，验证平移方向与量级。"""
    # 标定：相机在头部末端前方约 (0, 0, 0.1m)
    T_head_cam = np.eye(4)
    T_head_cam[0, 3] = 0.0   # mm
    T_head_cam[1, 3] = 0.0
    T_head_cam[2, 3] = 100.0  # 100mm 前方
    hk = HeadCameraToBase(T_head_cam, link_length_mm=162.0)

    T = hk.compute_cam2robot_meters(0.0, 0.0, 0.0)
    assert T.shape == (4, 4)
    # 相机原点在 base 下应为 (0, 0, 0.162+0.100) m
    np.testing.assert_allclose(T[:3, 3], [0.0, 0.0, 0.262], atol=1e-9)


def test_head_pitch_rotation():
    """头部俯仰 90° 时，相机前方点应转到 base 下方。"""
    T_head_cam = np.eye(4)
    hk = HeadCameraToBase(T_head_cam, link_length_mm=162.0)
    T = hk.compute_cam2robot_meters(0.0, 90.0, 0.0)
    # 相机系下前方 1m 的点
    p_cam = np.array([0.0, 0.0, 1.0])
    p_robot = T[:3, :3] @ p_cam + T[:3, 3]
    # 按 R=Rz(yaw)@Ry(pitch)@Rx(roll) 约定，pitch=+90° 后相机 z 轴指向 base 的 +x 方向
    assert p_robot[0] > 0.9, f"预期 x 为正，实际 {p_robot}"
    np.testing.assert_allclose(p_robot[1], 0.0, atol=1e-9)
    np.testing.assert_allclose(p_robot[2], 0.162, atol=1e-6)


def test_euler_roundtrip():
    """欧拉角 -> 旋转矩阵 保持右手正交。"""
    R = euler_to_rotation_matrix(10.0, -15.0, 30.0)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-12)


def test_depth_centroid_to_robot():
    """模拟深度重心：相机系 (0,0,0.5)m，零头部角 -> base 下 (0,0,0.662)m。"""
    T_head_cam = np.eye(4)
    T_head_cam[2, 3] = 0.0
    hk = HeadCameraToBase(T_head_cam, link_length_mm=162.0)
    T = hk.compute_cam2robot_meters(0.0, 0.0, 0.0)
    p_cam = np.array([0.0, 0.0, 0.5])
    p_robot = T[:3, :3] @ p_cam + T[:3, 3]
    np.testing.assert_allclose(p_robot, [0.0, 0.0, 0.662], atol=1e-9)


def test_real_calibration_file():
    """用工程自带的标定文件验证可加载且输出有限。"""
    calib = Path(__file__).resolve().parent.parent / "calibration" / "camera_to_head_connector_result.yaml"
    hk = HeadCameraToBase.from_yaml(calib, link_length_mm=162.0)
    for pitch in (0.0, 15.0, 30.0, -15.0):
        T = hk.compute_cam2robot_meters(0.0, pitch, 0.0)
        assert np.all(np.isfinite(T)), f"pitch={pitch} 输出非有限"
        assert T.shape == (4, 4)
        print(f"  pitch={pitch:6.1f}deg  T_base_cam 平移(m)={np.round(T[:3, 3], 4)}")


if __name__ == "__main__":
    test_identity_transform()
    test_head_pitch_rotation()
    test_euler_roundtrip()
    test_depth_centroid_to_robot()
    print("[test] 标定文件验证:")
    test_real_calibration_file()
    print("\n所有几何测试通过 ✔")
