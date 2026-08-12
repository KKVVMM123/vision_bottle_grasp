"""头部相机系到 base 系的变换（与 free_calib 约定一致）。

T_base_cam = Trans(0, 0, link_length_mm) @ R(roll, pitch, yaw) @ T_head_cam
R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
平移单位：mm；视觉流水线使用时再换算为 m。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


@dataclass
class TransformResult:
    matrix: np.ndarray
    translation_xyz_mm: np.ndarray
    euler_angles_xyz_deg: np.ndarray


def euler_to_rotation_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    rx_mat = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    ry_mat = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rz_mat = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rz_mat @ ry_mat @ rx_mat


def rotation_matrix_to_euler_xyz_deg(rotation: np.ndarray) -> np.ndarray:
    sy = -rotation[2, 0]
    cy = np.sqrt(max(0.0, 1.0 - sy * sy))
    if cy < 1e-8:
        rx = np.arctan2(-rotation[1, 2], rotation[1, 1])
        ry = np.arcsin(np.clip(sy, -1.0, 1.0))
        rz = 0.0
    else:
        rx = np.arctan2(rotation[2, 1] / cy, rotation[2, 2] / cy)
        ry = np.arcsin(np.clip(sy, -1.0, 1.0))
        rz = np.arctan2(rotation[1, 0] / cy, rotation[0, 0] / cy)
    return np.rad2deg([rx, ry, rz])


def make_transform(rotation: np.ndarray, translation_mm: Sequence[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation_mm, dtype=np.float64)
    return transform


def load_matrix_from_yaml(yaml_path: str | Path) -> np.ndarray:
    if yaml is None:
        raise ImportError("读取 yaml 需要 PyYAML，请安装: pip install PyYAML")

    path = Path(yaml_path)
    with path.open(encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict):
        raise ValueError(f"{path} 内容必须是 yaml 字典")

    if "a. matrix" in data:
        matrix = data["a. matrix"]
    elif "matrix" in data:
        matrix = data["matrix"]
    else:
        raise ValueError(f"未在 {path} 中找到 4x4 matrix 字段")

    return np.asarray(matrix, dtype=np.float64)


class HeadCameraToBase:
    """根据头部电机角计算相机系到 base 系的变换。"""

    def __init__(self, T_head_cam: np.ndarray, link_length_mm: float) -> None:
        transform = np.asarray(T_head_cam, dtype=np.float64)
        if transform.shape != (4, 4):
            raise ValueError("T_head_cam 必须是 4x4 矩阵")
        self.T_head_cam = transform
        self.link_length_mm = float(link_length_mm)

    @classmethod
    def from_yaml(cls, calib_yaml: str | Path, link_length_mm: float) -> HeadCameraToBase:
        return cls(load_matrix_from_yaml(calib_yaml), link_length_mm)

    def compute(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> TransformResult:
        rotation_head = euler_to_rotation_matrix(roll_deg, pitch_deg, yaw_deg)
        transform_base_head = make_transform(rotation_head, (0.0, 0.0, self.link_length_mm))
        transform_base_cam = transform_base_head @ self.T_head_cam
        return TransformResult(
            matrix=transform_base_cam,
            translation_xyz_mm=transform_base_cam[:3, 3].copy(),
            euler_angles_xyz_deg=rotation_matrix_to_euler_xyz_deg(transform_base_cam[:3, :3]),
        )

    def compute_cam2robot_meters(
        self, roll_deg: float, pitch_deg: float, yaw_deg: float
    ) -> np.ndarray:
        """返回 float32 的 T_base_cam，平移单位为米（供视觉流水线使用）。"""
        result = self.compute(roll_deg, pitch_deg, yaw_deg)
        cam2robot = result.matrix.astype(np.float32).copy()
        cam2robot[:3, 3] /= 1000.0
        return cam2robot
