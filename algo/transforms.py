"""位姿/旋转工具 —— 从 reBot-DevArm-Grasp 的 utils/transforms.py 摘出。

只保留标定链路真正用到的函数，函数体与上游逐字一致：
  quat_to_mat4(x,y,z,qx,qy,qz,qw)  把话题里的四元数位姿转成 4x4
  rotation_matrix_to_euler_zyx(R)  intrinsic ZYX 欧拉角，仅用于打印 / OSD
  _nearest_rotation_matrix(R)      上面那个的辅助：SVD 投影回 SO(3)

另加一个 rpy_to_quat：与 genisys examples/movel_demo.py 的 zyx_to_quat 逐字等价，
供 http_rpc.movej_pose 把 ZYX 欧拉角转成四元数下发（约定集中在本文档，便于审计）。

未移植的内容（抓取相关，标定用不到）：
  pose6d_to_mat4 / mat4_to_pose6d / canonicalize_parallel_gripper_tcp_rotation /
  grasp_axes_to_rebot_tcp_rotation / transform_grasp_pose_to_base* 等。

原文件：utils/transforms.py（第 15-31、75-100、110-124 行）
"""
import math

import numpy as np


def rpy_to_quat(roll: float, pitch: float, yaw: float):
    """intrinsic ZYX 欧拉角（弧度）→ Hamilton 四元数 (w, x, y, z)。

    对应 R = Rz(yaw) · Ry(pitch) · Rx(roll)，与 genisys 的 zyx_to_quat 等价，
    也与本文件的 rotation_matrix_to_euler_zyx 互逆。
    """
    r, p, y = roll / 2.0, pitch / 2.0, yaw / 2.0
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _nearest_rotation_matrix(R: np.ndarray) -> np.ndarray:
    """Project a near-rotation matrix onto SO(3)."""
    R = np.asarray(R, dtype=np.float64)
    if R.shape != (3, 3):
        raise ValueError(f"rotation matrix must be (3, 3), got {R.shape}")

    if not np.all(np.isfinite(R)):
        raise ValueError("rotation matrix contains non-finite values")

    U, _, Vt = np.linalg.svd(R)
    R_ortho = U @ Vt
    if np.linalg.det(R_ortho) < 0.0:
        U[:, -1] *= -1.0
        R_ortho = U @ Vt
    return R_ortho.astype(np.float64)



def quat_to_mat4(x, y, z, qx, qy, qz, qw) -> np.ndarray:
    """
    Convert translation and quaternion to a 4x4 homogeneous transform.

    Args:
        x, y, z: translation in meters.
        qx, qy, qz, qw: Hamilton quaternion.

    Returns:
        T: (4, 4) numpy array
    """
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    R = np.array([
        [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
        [  2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),   2*(qy*qz - qx*qw)],
        [  2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = [x, y, z]
    return T



def rotation_matrix_to_euler_zyx(R: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to intrinsic ZYX Euler angles."""
    R = _nearest_rotation_matrix(R)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0.0
    return np.array([rx, ry, rz], dtype=np.float64)


