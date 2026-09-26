import requests
import json

from algo.transforms import rpy_to_quat

# ---------------- HTTP RPC 封装 ----------------

class RpcClient:
    """AimRT http+json RPC 客户端（MoveJ / StartRecord / StopRecord）。"""

    def __init__(self, base_url: str, timeout: float):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout

    def _post(self, service: str, method: str, payload: dict):
        url = f"{self.base_url}/rpc/{service}/{method}"
        try:
            resp = requests.post(url, json=payload, timeout=self.timeout)
            resp.raise_for_status()
            result = resp.json()
            status_info = result.get("error") or result.get("common_rsp") or result
            code = status_info.get("code", -1)
            if int(code) == 0:
                print(f"\n✅ {method} succeeded!")
                return True
            else:
                print(f"\n❌ {method} failed (code={code}): \nResponse: {json.dumps(result, indent=2)}")
        except requests.exceptions.ConnectionError:
            print(f"\n❌ {method} 无法连接到 GeniArm (127.0.0.1:50080)，请确认程序已启动。")
        except requests.exceptions.Timeout:
            print(f"\n❌ {method} 请求超时，轨迹执行时间过长。")
        except requests.exceptions.RequestException as e:
            print(f"\n❌ {method} 请求失败: {e}")


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
        v/a 是**关节空间**的速度与加速度，单位 rad/s 与 rad/s²
        （默认 0.8 / 0.4，取自 genisys examples；注意 MoveL 用的是 m/s，别混）。
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