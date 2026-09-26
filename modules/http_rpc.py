import requests

from algo.transforms import rpy_to_quat

# ---------------- HTTP RPC 封装 ----------------

class RpcClient:
    """AimRT http+json RPC 客户端（MoveJ / StartRecord / StopRecord）。"""

    def __init__(self, base_url: str, timeout: float):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.last_error = ""

    def _post(self, service: str, method: str, payload: dict) -> bool:
        """成功返回 True；失败返回 False，原因写入 self.last_error。"""
        url = f"{self.base_url}/rpc/{service}/{method}"
        self.last_error = ""
        try:
            resp = requests.post(url, json=payload, timeout=self.timeout)
            resp.raise_for_status()
            result = resp.json()
            status_info = result.get("error") or result.get("common_rsp") or result
            code = status_info.get("code", -1)
            if int(code) == 0:
                return True
            self.last_error = f"code={code} {status_info.get('msg', '')}".strip()
        except requests.exceptions.ConnectionError:
            self.last_error = f"cannot connect to {self.base_url}"
        except requests.exceptions.Timeout:
            self.last_error = f"timeout after {self.timeout}s"
        except requests.exceptions.RequestException as e:
            self.last_error = str(e)
        return False


    # --- 机械臂运动（MoveJ 为阻塞语义：任务完成才返回） ---
    def movej(self, q: list, v: float, a: float):
        """MoveJ 到关节角 q（弧度），阻塞至到位后返回响应。"""
        return self._post(
            "genisys_msgs/srv", "MoveJ",
            {"position": {"position": [float(x) for x in q]},
             "v": float(v), "a": float(a)})

    # --- 机械臂运动（MoveJPose 为阻塞语义：任务完成才返回） ---
    def movej_pose(self, x: float, y: float, z: float,
                   roll: float, pitch: float, yaw: float,
                   v: float = 0.8, a: float = 0.4):
        """MoveJPose 到指定位姿（阻塞，返回 True / None）。

        rpy 为 intrinsic ZYX 欧拉角，单位弧度。
        v/a 是关节空间的速度与加速度，单位 rad/s 与 rad/s²。
        """
        w, qx, qy, qz = rpy_to_quat(roll, pitch, yaw)
        return self._post(
            "genisys_msgs/srv", "MoveJPose",
            {
                "pose": {"position": {"x": float(x), "y": float(y), "z": float(z)},
                         "orientation": {"x": qx, "y": qy, "z": qz, "w": w}},
                "v": float(v),
                "a": float(a)
            })

    # --- record 插件 ---
    def start_record(self, action_name: str):
        return self._post(
            "aimrt.protocols.record_playback_plugin.RecordPlaybackService",
            "StartRecord",
            {"action_name": action_name,
             "preparation_duration_s": 0, "record_duration_s": 0})

    def stop_record(self, action_name: str):
        return self._post(
            "aimrt.protocols.record_playback_plugin.RecordPlaybackService",
            "StopRecord",
            {"action_name": action_name})