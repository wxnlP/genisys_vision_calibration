from dataclasses import dataclass

import requests

from algo.transforms import rpy_to_quat


@dataclass(frozen=True)
class MoveResult:
    """运动 RPC 的受理结果（不是完成结果）。"""

    accepted: bool
    task_id: int = 0
    duration_ms: float = 0.0
    code: int = -1
    msg: str = ""

    @property
    def ok(self) -> bool:
        return self.accepted


class RpcClient:
    """AimRT http+json RPC 客户端（MoveJ / MoveJPose / Record）。"""

    SERVICE_MOVE = "genisys_msgs/srv"
    SERVICE_RECORD = "aimrt.protocols.record_playback_plugin.RecordPlaybackService"

    def __init__(self, base_url: str, timeout: float):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.last_error = ""

    # ---------------- 底层 ----------------

    def _post(self, service: str, method: str, payload: dict):
        """POST 并返回解析后的 JSON；网络 / 协议错误返回 None，原因写入 last_error。"""
        url = f"{self.base_url}/rpc/{service}/{method}"
        self.last_error = ""
        try:
            resp = requests.post(url, json=payload, timeout=self.timeout)
            resp.raise_for_status()
            result = resp.json()
        except requests.exceptions.ConnectionError:
            self.last_error = f"cannot connect to {self.base_url}"
            return None
        except requests.exceptions.Timeout:
            self.last_error = f"timeout after {self.timeout}s"
            return None
        except requests.exceptions.RequestException as e:
            self.last_error = str(e)
            return None
        except ValueError as e:   # 非 JSON 响应
            self.last_error = f"invalid JSON response: {e}"
            return None
        if not isinstance(result, dict):
            self.last_error = f"unexpected response: {result!r}"
            return None
        return result

    def _post_ok(self, service: str, method: str, payload: dict) -> bool:
        """兼容 error / common_rsp 两种状态字段的布尔封装（record 等旧接口用）。"""
        result = self._post(service, method, payload)
        if result is None:
            return False
        status_info = result.get("error") or result.get("common_rsp") or result
        if not isinstance(status_info, dict):
            status_info = result
        code = status_info.get("code", -1)
        if int(code) == 0:
            return True
        self.last_error = f"code={code} {status_info.get('msg', '')}".strip()
        return False

    # ---------------- 运动 ----------------

    def _move(self, method: str, payload: dict) -> MoveResult:
        """发起运动请求并解析受理结果。"""
        result = self._post(self.SERVICE_MOVE, method, payload)
        if result is None:
            return MoveResult(False, msg=self.last_error)

        status_info = result.get("error") or result.get("common_rsp") or {}
        if not isinstance(status_info, dict):
            status_info = {}
        try:
            code = int(status_info.get("code", -1))
        except (TypeError, ValueError):
            code = -1
        msg = str(status_info.get("msg", "") or "")
        try:
            task_id = int(result.get("task_id", 0) or 0)
        except (TypeError, ValueError):
            task_id = 0
        try:
            duration_ms = float(result.get("duration_ms", 0.0) or 0.0)
        except (TypeError, ValueError):
            duration_ms = 0.0

        accepted = code == 0
        if not accepted:
            self.last_error = f"code={code} {msg}".strip()
        return MoveResult(accepted, task_id, duration_ms, code, msg)

    def movej(self, q: list, v: float, a: float) -> MoveResult:
        """MoveJ 到关节角 q（弧度）。受理即返回，不等动作完成。"""
        return self._move(
            "MoveJ",
            {"position": {"position": [float(x) for x in q]},
             "v": float(v), "a": float(a)})

    def movej_pose(self, x: float, y: float, z: float,
                   roll: float, pitch: float, yaw: float,
                   v: float = 0.8, a: float = 0.4) -> MoveResult:
        """MoveJPose 到指定位姿。受理即返回，不等动作完成。

        rpy 为 intrinsic ZYX 欧拉角，单位弧度；目标位姿是 TCP 位姿。
        v/a 是关节空间的速度与加速度，单位 rad/s 与 rad/s²。
        """
        w, qx, qy, qz = rpy_to_quat(roll, pitch, yaw)
        return self._move(
            "MoveJPose",
            {
                "pose": {"position": {"x": float(x), "y": float(y), "z": float(z)},
                         "orientation": {"x": qx, "y": qy, "z": qz, "w": w}},
                "v": float(v),
                "a": float(a)
            })

    # ---------------- record 插件 ----------------

    def start_record(self, action_name: str) -> bool:
        return self._post_ok(
            self.SERVICE_RECORD, "StartRecord",
            {"action_name": action_name,
             "preparation_duration_s": 0, "record_duration_s": 0})

    def stop_record(self, action_name: str) -> bool:
        return self._post_ok(
            self.SERVICE_RECORD, "StopRecord",
            {"action_name": action_name})
