"""手眼标定流程模块（Eye-in-Hand / TSAI）。

订阅 /geniarm/pose_current 获取 TCP 位姿，用 MoveJPose 遍历预设点位；
每个点位在 ArUco 稳定后采集一组 (T_gripper2base, T_marker2cam)，
遍历结束（或中断）时求解并存盘。

运行：./start_hand_eye.sh
输出：cfg/calibration/<camera.type>/hand_eye.npz
"""
import threading
import time
from pathlib import Path

import aimrt_py as aimrt
import cv2
import yaml

from modules.http_rpc import RpcClient
from protocols.pose_pb2 import PoseStamped
from algo.hand_eye import CalibMode, HandEyeCalibrator
from algo.transforms import quat_to_mat4, rotation_matrix_to_euler_zyx
from drivers.camera import make_camera


# ==========================================================
# 预设标定位姿（笛卡尔，米 / 弧度）：(x, y, z, roll, pitch, yaw)
# 姿态为 intrinsic ZYX 欧拉角；pitch > 0 表示末端朝下对着桌上的 ArUco。
# ==========================================================
CALIB_POSES_XYZ = [
    # Center region, large pitch, yaw sweep.
    (0.28, -0.16, 0.26, -0.30, 0.80, -0.90),
    (0.28, -0.08, 0.26,  0.30, 0.80, -0.45),
    (0.28,  0.00, 0.26, -0.30, 0.80,  0.00),
    (0.28,  0.08, 0.26,  0.30, 0.80,  0.45),
    (0.28,  0.16, 0.26, -0.30, 0.80,  0.90),
    # Center region, medium pitch.
    (0.27, -0.16, 0.31,  0.30, 0.55, -0.90),
    (0.27, -0.08, 0.31, -0.30, 0.55, -0.45),
    (0.27,  0.00, 0.31,  0.30, 0.55,  0.00),
    (0.27,  0.08, 0.31, -0.30, 0.55,  0.45),
    (0.27,  0.16, 0.31,  0.30, 0.55,  0.90),
    # Center region, small pitch.
    (0.26, -0.14, 0.34, -0.40, 0.35, -0.80),
    (0.26,  0.00, 0.34,  0.40, 0.35,  0.00),
    (0.26,  0.14, 0.34, -0.40, 0.35,  0.80),
    # Forward region, yaw +/- 1 rad.
    (0.37,  0.00, 0.27,  0.00, 0.65,  0.00),
    (0.37,  0.00, 0.27,  0.00, 0.65,  1.00),
    (0.37,  0.00, 0.27,  0.00, 0.65, -1.00),
    (0.37,  0.08, 0.27,  0.50, 0.65,  0.50),
    (0.37, -0.08, 0.27, -0.50, 0.65, -0.50),
    # Lateral poses with larger x.
    (0.33,  0.18, 0.27,  0.50, 0.50,  0.55),
    (0.33, -0.18, 0.27, -0.50, 0.50, -0.55),
    # Lateral poses with larger y.
    (0.20,  0.22, 0.28,  0.60, 0.40,  0.70),
    (0.20, -0.22, 0.28, -0.60, 0.40, -0.70),
    # Diagonal forward-y poses with large roll.
    (0.24, -0.20, 0.31,  0.70, 0.45, -1.00),
    (0.24,  0.20, 0.31, -0.70, 0.45,  1.00),
    (0.25, -0.15, 0.29, -0.60, 0.62, -0.50),
    (0.25,  0.15, 0.29,  0.60, 0.62,  0.50),
    # High poses.
    (0.21, -0.09, 0.40, -0.40, 0.25, -0.60),
    (0.21,  0.00, 0.40,  0.40, 0.25,  0.00),
    (0.21,  0.09, 0.40, -0.40, 0.25,  0.60),
    (0.20, -0.09, 0.40,  0.40, 0.28, -0.60),
    (0.20,  0.09, 0.40, -0.40, 0.28,  0.60),
    # Low poses.
    (0.30, -0.10, 0.24,  0.40, 0.70, -0.70),
    (0.30,  0.00, 0.24,  0.00, 0.75,  0.00),
    (0.30,  0.10, 0.24, -0.40, 0.70,  0.70),
    # Roll extremes.
    (0.26,  0.12, 0.30,  0.80, 0.50,  0.30),
    (0.26, -0.12, 0.30, -0.80, 0.50, -0.30),
    # Large roll and yaw combinations for rotation diversity.
    (0.29, -0.10, 0.28,  0.90, 0.60, -0.40),
    (0.29,  0.10, 0.28, -0.90, 0.60,  0.40),
    (0.28, -0.18, 0.30,  0.85, 0.55, -0.80),
    (0.28,  0.18, 0.30, -0.85, 0.55,  0.80),
    # Forward reach with different roll/yaw combinations.
    (0.35, -0.12, 0.30,  0.60, 0.58, -0.70),
    (0.35,  0.12, 0.30, -0.60, 0.58,  0.70),
    (0.34, -0.06, 0.28,  0.40, 0.72,  0.80),
    (0.34,  0.06, 0.28, -0.40, 0.72, -0.80),
    # High poses with large roll.
    (0.22, -0.14, 0.38,  0.75, 0.32, -0.70),
    (0.22,  0.14, 0.38, -0.75, 0.32,  0.70),
    # Lateral poses with large pitch and opposite yaw.
    (0.31,  0.20, 0.27,  0.30, 0.68,  0.95),
    (0.31, -0.20, 0.27, -0.30, 0.68, -0.95),
    # Mid-range poses covering roll, pitch, and yaw.
    (0.30,  0.05, 0.32,  0.75, 0.42,  0.65),
    (0.30, -0.05, 0.32, -0.75, 0.42, -0.65),
]

MIN_CALIB_SAMPLES = 5
AUTO_MARKER_TIMEOUT_S = 2.5
AUTO_MARKER_STABLE_FRAMES = 4
FRAME_FLUSH_AFTER_MOVE = 5

WIN = "Hand-Eye Calibration  (Q / ESC to quit)"


class HandEyeModule(aimrt.ModuleBase):
    def __init__(self):
        super().__init__()
        # AimRT 对象
        self.core = aimrt.CoreRef()
        self.logger = aimrt.LoggerRef()
        self.work_executor = aimrt.ExecutorRef()
        self.sub_cart_pose = aimrt.SubscriberRef()

        self.rpc = RpcClient("http://127.0.0.1:50080", 30)

        # 最新 TCP 位姿（4x4，base 系）；首帧前为 None
        self.tcp_pose = None

        self.running = False
        self._thread = None

        # 相机 / 标定器（Initialize 里构造，线程里打开）
        self._cam = None
        self._calibrator = None
        self._save_path = None

        # 自动遍历状态
        self._auto = {
            "idx": 0,
            "pose_idx": None,
            "phase": "idle",        # idle = 该发下一条运动指令
            "timeout_at": 0.0,
            "stable_frames": 0,
            "finished": False,
        }

    # ---------- AimRT 生命周期 ----------

    def Info(self) -> aimrt.ModuleInfo:
        info = aimrt.ModuleInfo()
        info.name = "HandEyeModule"
        return info

    def Initialize(self, core: aimrt.CoreRef) -> bool:
        self.core = core
        self.logger = self.core.GetLogger()

        file_path = self.core.GetConfigurator().GetConfigFilePath()
        with open(file_path, "r") as f:
            config = yaml.safe_load(f)
        work_executor_name = str(config["work_executor"])
        sub_cart_pose_topic = str(config["sub_cart_pose_topic"])
        cam_cfg = config["camera"]
        aruco_cfg = config["calibration"]["aruco"]
        he_method = config["calibration"].get("hand_eye_method", "TSAI")

        self.work_executor = self.core.GetExecutorManager().GetExecutor(work_executor_name)
        self.sub_cart_pose = self.core.GetChannelHandle().GetSubscriber(sub_cart_pose_topic)
        aimrt.Subscribe(self.sub_cart_pose, PoseStamped, self.cart_pose_callback)

        self._cam = make_camera({"camera": cam_cfg})
        self._cam.setup_aruco(
            marker_length_m=aruco_cfg["marker_length_m"],
            dict_id=aruco_cfg.get("dict_id", 0),
            target_marker_id=aruco_cfg.get("target_marker_id"),
        )

        self._calibrator = HandEyeCalibrator(CalibMode.EYE_IN_HAND, method=he_method)
        self._save_path = (
            Path(__file__).resolve().parent.parent
            / "cfg" / "calibration" / str(cam_cfg["type"]) / "hand_eye.npz"
        )

        print(f"[Init] camera={cam_cfg['type']}  solver={he_method}  "
              f"aruco={aruco_cfg['marker_length_m'] * 100:.1f}cm  "
              f"poses={len(CALIB_POSES_XYZ)}  output={self._save_path}")
        return True

    def Start(self) -> bool:
        self.running = True
        self._thread = threading.Thread(target=self._calib_loop, name="hand-eye-calib", daemon=True)
        self._thread.start()
        return True

    def Shutdown(self):
        try:
            self.running = False
            if self._thread is not None:
                self._thread.join(timeout=3.0)
            if self._cam is not None:
                self._cam.close()
            aimrt.info(self.logger, "HandEyeModule shutdown.")
        except Exception as e:
            aimrt.error(self.logger, f"Error in Shutdown: {e}")

    # ---------- 位姿订阅 ----------

    def cart_pose_callback(self, msg):
        if not self.running:
            return
        p, q = msg.pose.position, msg.pose.orientation
        self.tcp_pose = quat_to_mat4(p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    # ---------- 标定主循环 ----------

    def _calib_loop(self):
        finish_reason = "normal finish"

        try:
            self._cam.open()
            print("[Camera] warming up...", end="", flush=True)
            self._cam.warm_up(20)
            print(" ready")
        except Exception as e:
            print(f"[Camera] 打开失败，标定线程退出: {e}")
            return

        gui = True
        try:
            cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
        except cv2.error:
            gui = False
            print("[UI] 无显示环境，转为无界面模式（只在终端输出）")

        try:
            while self.running:
                bgr, _ = self._cam.get_frame()
                if bgr is None:
                    continue

                marker = self._cam.detect_aruco(bgr)
                if self._tick(marker):
                    finish_reason = "auto traversal completed"

                if gui:
                    vis = self._cam.draw_aruco(bgr)
                    self._draw_osd(vis, marker)
                    cv2.imshow(WIN, vis)
                    if cv2.waitKey(30) & 0xFF in (ord('q'), ord('Q'), 27):
                        finish_reason = "window exit"
                        break
                    if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                        finish_reason = "window closed"
                        break
                else:
                    time.sleep(0.03)   # 无界面时按 ~30fps 节流

                if self._auto["finished"]:
                    break
        except KeyboardInterrupt:
            finish_reason = "Ctrl+C interrupt"
        finally:
            if gui:
                cv2.destroyAllWindows()
            self._cam.close()
            self._compute_and_save(finish_reason)

    # ---------- 自动遍历状态机 ----------

    def _start_next_pose(self) -> bool:
        """发下一条运动指令（阻塞）。返回 True 表示全部点位走完。"""
        total = len(CALIB_POSES_XYZ)
        while self._auto["idx"] < total:
            idx = self._auto["idx"]
            x, y, z, roll, pitch, yaw = CALIB_POSES_XYZ[idx]
            print(f"\n[Auto] Pose {idx + 1}/{total}: "
                  f"pos=({x:.2f},{y:.2f},{z:.2f}) rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})")

            ok = self.rpc.movej_pose(x, y, z, roll, pitch, yaw)
            if ok:
                self._auto["pose_idx"] = idx
                self._auto["phase"] = "searching"
                self._auto["timeout_at"] = time.monotonic() + AUTO_MARKER_TIMEOUT_S
                self._auto["stable_frames"] = 0
                self._flush_frames()
                return False

            print(f"[Auto] Pose {idx + 1}/{total} 不可达 / 运动失败，跳过")
            self._auto["idx"] += 1

        self._auto["finished"] = True
        print("\n[Auto] 所有预设点位已走完")
        return True

    def _tick(self, marker) -> bool:
        auto = self._auto
        if auto["finished"]:
            return True
        if auto["phase"] == "idle":
            return self._start_next_pose()

        total = len(CALIB_POSES_XYZ)
        pose_no = (auto["pose_idx"] or 0) + 1

        if marker is not None:
            auto["stable_frames"] += 1
            if auto["stable_frames"] >= AUTO_MARKER_STABLE_FRAMES:
                self._capture_sample(marker, f"auto pose {pose_no}/{total}")
                auto["idx"] += 1
                auto["phase"] = "idle"
                auto["stable_frames"] = 0
                return self._start_next_pose()
        else:
            auto["stable_frames"] = 0

        if time.monotonic() >= auto["timeout_at"]:
            print(f"[Auto] Pose {pose_no}/{total} 等待 ArUco 超时，跳过")
            auto["idx"] += 1
            auto["phase"] = "idle"
            auto["stable_frames"] = 0
            return self._start_next_pose()

        return False

    def _flush_frames(self):
        """丢弃运动期间积压的旧帧。"""
        for _ in range(FRAME_FLUSH_AFTER_MOVE):
            self._cam.get_frame()

    # ---------- 采样与求解 ----------

    def _capture_sample(self, marker, source: str) -> bool:
        if self.tcp_pose is None:
            print("  [Skip] 尚未收到 TCP 位姿，跳过本样本")
            return False

        T_g2b = self.tcp_pose
        c = self._calibrator
        print(f"\n[Sample {c.n_samples + 1}] {source}")
        print(f"  ArUco: x={marker.T_marker2cam[0, 3]:.3f} "
              f"y={marker.T_marker2cam[1, 3]:.3f} "
              f"z={marker.T_marker2cam[2, 3]:.3f} m")
        t = T_g2b[:3, 3]
        print(f"  TCP  : x={t[0]:.4f} y={t[1]:.4f} z={t[2]:.4f} m")
        c.add_sample(T_g2b, marker.T_marker2cam)
        print(f"  [OK] 累计样本 {c.n_samples}")
        return True

    def _compute_and_save(self, reason: str) -> bool:
        c = self._calibrator
        print(f"\n[Finish] {reason}")
        if c is None or c.n_samples < MIN_CALIB_SAMPLES:
            n = 0 if c is None else c.n_samples
            print(f"[Result] 样本不足（{n} < {MIN_CALIB_SAMPLES}），未求解")
            return False

        print(f"[Result] 用 {c.n_samples} 个样本求解...")
        try:
            result = c.calibrate(min_samples=MIN_CALIB_SAMPLES)
            HandEyeCalibrator.save(result, self._save_path)
            t = result.T_result[:3, 3]
            print(f"[Result] T_cam2gripper 平移: x={t[0]:.4f} y={t[1]:.4f} z={t[2]:.4f} m")
            print(f"[Result] 已保存到 {self._save_path}")
            if c.n_samples < 15:
                print("[Result] 提示：样本 < 15，精度有限，建议再采一些")
            return True
        except Exception as e:
            print(f"[Result] 求解失败: {e}")
            return False

    # ---------- OSD ----------

    def _draw_osd(self, vis, marker):
        def osd(text, y, color=(220, 220, 220)):
            cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

        n = self._calibrator.n_samples
        if marker is not None:
            osd(f"[ID={marker.id}] z={marker.T_marker2cam[2, 3]:.3f}m  samples:{n}", 28, (80, 220, 80))
        else:
            osd(f"No marker  samples:{n}", 28, (80, 80, 220))

        auto = self._auto
        total = len(CALIB_POSES_XYZ)
        if auto["finished"]:
            status = "all poses done"
        elif auto["phase"] == "idle":
            status = "moving..."
        else:
            remain = max(0.0, auto["timeout_at"] - time.monotonic())
            status = (f"pose {(auto['pose_idx'] or 0) + 1}/{total} marker stable "
                      f"{auto['stable_frames']}/{AUTO_MARKER_STABLE_FRAMES}  wait {remain:.1f}s")
        osd(f"AUTO: {status}", 50, (180, 180, 60))

        if self.tcp_pose is not None:
            t = self.tcp_pose[:3, 3]
            r, p, y = rotation_matrix_to_euler_zyx(self.tcp_pose[:3, :3])
            osd(f"TCP x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} "
                f"rpy=[{r:+.2f} {p:+.2f} {y:+.2f}]", 72)

        filled = min(n, 15) * (400 // 15)
        cv2.rectangle(vis, (10, 86), (10 + filled, 98), (0, 200, 100), -1)
        cv2.rectangle(vis, (10, 86), (410, 98), (160, 160, 160), 1)
        cv2.putText(vis, f"{n}/15", (10, 113), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
