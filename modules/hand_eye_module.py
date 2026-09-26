"""手眼标定流程模块（Eye-in-Hand / TSAI）。

订阅 /geniarm/pose_current 获取 TCP 位姿，用 MoveJPose 遍历预设点位；
每个点位在 ArUco 稳定后采集一组 (T_gripper2base, T_marker2cam)，
遍历结束（或中断）时求解并存盘。

运行：./start_hand_eye.sh
输出：cfg/calibration/<camera.type>/hand_eye.npz
"""
import time
from pathlib import Path

import aimrt_py as aimrt
import cv2
import rerun as rr
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

        # 运行模式：normal / vision_only / motion_only
        self._mode = "normal"
        self._vision_found = False

        # 相机 / 标定器（Initialize 里构造，线程里打开）
        self._cam = None
        self._calibrator = None
        self._save_path = None

        # rerun（画面显示）
        self._rr_enabled = False
        self._rr_port = 9876
        self._rr_quality = 80
        self._rr_every = 1
        self._rr_save = ""
        self._frame_i = 0

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
        self._mode = str(config.get("mode", "normal"))
        cam_cfg = config["camera"]
        aruco_cfg = config["calibration"]["aruco"]
        he_method = config["calibration"].get("hand_eye_method", "TSAI")
        rr_cfg = config.get("rerun") or {}
        self._rr_enabled = bool(rr_cfg.get("enabled", True))
        self._rr_port = int(rr_cfg.get("grpc_port", 9876))
        self._rr_quality = int(rr_cfg.get("jpeg_quality", 80))
        self._rr_every = max(1, int(rr_cfg.get("log_every_n_frames", 1)))
        self._rr_save = str(rr_cfg.get("save_rrd", "") or "")

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

        aimrt.info(self.logger,
                   f"[init] mode={self._mode} camera={cam_cfg['type']} solver={he_method} "
                   f"aruco={aruco_cfg['marker_length_m'] * 100:.1f}cm "
                   f"poses={len(CALIB_POSES_XYZ)}")
        return True

    def Start(self) -> bool:
        self.running = True
        self.work_executor.Execute(self._calib_loop)
        return True

    def Shutdown(self):
        try:
            self.running = False
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

        if self._rr_enabled:
            try:
                rr.init("hand_eye_calib", spawn=False)
                rr.serve_grpc(grpc_port=self._rr_port)
                if self._rr_save:
                    rr.save(self._rr_save)
                aimrt.info(self.logger,
                           f"[rerun] serve_grpc port={self._rr_port} jpeg_q={self._rr_quality} "
                           f"every={self._rr_every}"
                           + (f" save={self._rr_save}" if self._rr_save else ""))
            except Exception as e:
                self._rr_enabled = False
                aimrt.warn(self.logger, f"[rerun] init failed, logging disabled: {e}")

        # motion_only：不开相机、不等标记、不采样
        if self._mode == "motion_only":
            self._motion_loop()
            return

        try:
            self._cam.open()
            self._cam.warm_up(20)
            aimrt.info(self.logger, "[camera] opened")
        except Exception as e:
            aimrt.error(self.logger, f"[camera] open failed: {e}")
            return

        try:
            while self.running:
                bgr, _ = self._cam.get_frame()
                if bgr is None:
                    continue

                marker = self._cam.detect_aruco(bgr)

                if self._mode == "normal":
                    if self._tick(marker):
                        finish_reason = "auto traversal completed"
                else:
                    self._report_vision(marker)

                self._log_frame(bgr, marker)

                if self._mode == "normal" and self._auto["finished"]:
                    break
        except KeyboardInterrupt:
            finish_reason = "Ctrl+C interrupt"
        finally:
            self._cam.close()
            if self._mode == "normal":
                self._compute_and_save(finish_reason)
            else:
                aimrt.info(self.logger, f"[finish] {finish_reason} (vision_only, no solve)")

    def _report_vision(self, marker):
        """vision_only：只在“识别到 / 丢失”发生变化时记录，避免刷屏。"""
        found = marker is not None
        if found == self._vision_found:
            return
        self._vision_found = found
        if found:
            t = marker.T_marker2cam[:3, 3]
            aimrt.info(self.logger,
                       f"[vision] marker id={marker.id} "
                       f"x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f}")
        else:
            aimrt.info(self.logger, "[vision] marker lost")

    def _motion_loop(self):
        """motion_only：只走 50 个点位，不开相机、不等标记、不采样。"""
        total = len(CALIB_POSES_XYZ)
        ok = skip = 0
        for idx, (x, y, z, roll, pitch, yaw) in enumerate(CALIB_POSES_XYZ):
            if not self.running:
                break
            aimrt.info(self.logger,
                       f"[pose {idx + 1}/{total}] x={x:.3f} y={y:.3f} z={z:.3f} "
                       f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})")
            if not self.rpc.movej_pose(x, y, z, roll, pitch, yaw):
                aimrt.warn(self.logger,
                           f"[pose {idx + 1}/{total}] move failed, skip: {self.rpc.last_error}")
                skip += 1
                continue
            ok += 1
            time.sleep(0.2)   # 等 pose_current 更新
            if self.tcp_pose is None:
                aimrt.warn(self.logger, f"[pose {idx + 1}/{total}] no pose_current")
                continue
            t = self.tcp_pose[:3, 3]
            r, p, yy = rotation_matrix_to_euler_zyx(self.tcp_pose[:3, :3])
            aimrt.info(self.logger,
                       f"[pose {idx + 1}/{total}] reached x={t[0]:+.4f} y={t[1]:+.4f} z={t[2]:+.4f} "
                       f"rpy=({r:+.3f},{p:+.3f},{yy:+.3f})")
        aimrt.info(self.logger, f"[motion] done ok={ok} skip={skip} total={total}")

    # ---------- 自动遍历状态机 ----------

    def _start_next_pose(self) -> bool:
        """发下一条运动指令（阻塞）。返回 True 表示全部点位走完。"""
        total = len(CALIB_POSES_XYZ)
        while self._auto["idx"] < total:
            idx = self._auto["idx"]
            x, y, z, roll, pitch, yaw = CALIB_POSES_XYZ[idx]
            aimrt.info(self.logger,
                       f"[pose {idx + 1}/{total}] x={x:.3f} y={y:.3f} z={z:.3f} "
                       f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})")

            ok = self.rpc.movej_pose(x, y, z, roll, pitch, yaw)
            if ok:
                self._auto["pose_idx"] = idx
                self._auto["phase"] = "searching"
                self._auto["timeout_at"] = time.monotonic() + AUTO_MARKER_TIMEOUT_S
                self._auto["stable_frames"] = 0
                self._flush_frames()
                return False

            aimrt.warn(self.logger,
                       f"[pose {idx + 1}/{total}] move failed, skip: {self.rpc.last_error}")
            self._auto["idx"] += 1

        self._auto["finished"] = True
        aimrt.info(self.logger, f"[poses] all {total} done")
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
            aimrt.warn(self.logger, f"[pose {pose_no}/{total}] marker timeout, skip")
            auto["idx"] += 1
            auto["phase"] = "idle"
            auto["stable_frames"] = 0
            return self._start_next_pose()

        return False

    def _flush_frames(self):
        """丢弃运动期间积压的旧帧。"""
        for _ in range(FRAME_FLUSH_AFTER_MOVE):
            self._cam.get_frame()

    def _log_frame(self, bgr, marker):
        """把标注后的画面推给 rerun。"""
        if not self._rr_enabled:
            return
        self._frame_i += 1
        if self._frame_i % self._rr_every:
            return
        vis = self._cam.draw_aruco(bgr)
        self._draw_osd(vis, marker)
        rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        rr.log("camera", rr.Image(rgb).compress(jpeg_quality=self._rr_quality))

    # ---------- 采样与求解 ----------

    def _capture_sample(self, marker, source: str) -> bool:
        if self.tcp_pose is None:
            aimrt.warn(self.logger, f"[{source}] no tcp pose yet, sample dropped")
            return False

        T_g2b = self.tcp_pose
        c = self._calibrator
        m = marker.T_marker2cam[:3, 3]
        t = T_g2b[:3, 3]
        c.add_sample(T_g2b, marker.T_marker2cam)
        aimrt.info(self.logger,
                   f"[sample {c.n_samples}] {source} "
                   f"marker=({m[0]:+.3f},{m[1]:+.3f},{m[2]:.3f}) "
                   f"tcp=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})")
        return True

    def _compute_and_save(self, reason: str) -> bool:
        c = self._calibrator
        aimrt.info(self.logger, f"[finish] {reason}")
        if c is None or c.n_samples < MIN_CALIB_SAMPLES:
            n = 0 if c is None else c.n_samples
            aimrt.warn(self.logger, f"[result] not enough samples: {n} < {MIN_CALIB_SAMPLES}")
            return False

        try:
            result = c.calibrate(min_samples=MIN_CALIB_SAMPLES)
            HandEyeCalibrator.save(result, self._save_path)
            t = result.T_result[:3, 3]
            aimrt.info(self.logger,
                       f"[result] n={c.n_samples} t_cam2gripper=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})")
            aimrt.info(self.logger, f"[result] saved to {self._save_path}")
            if c.n_samples < 15:
                aimrt.warn(self.logger, "[result] n<15, accuracy limited")
            return True
        except Exception as e:
            aimrt.error(self.logger, f"[result] solve failed: {e}")
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

        if self._mode != "normal":
            osd(self._mode.upper(), 50, (180, 180, 60))
        else:
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

            filled = min(n, 15) * (400 // 15)
            cv2.rectangle(vis, (10, 86), (10 + filled, 98), (0, 200, 100), -1)
            cv2.rectangle(vis, (10, 86), (410, 98), (160, 160, 160), 1)
            cv2.putText(vis, f"{n}/15", (10, 113), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        if self.tcp_pose is not None:
            t = self.tcp_pose[:3, 3]
            r, p, y = rotation_matrix_to_euler_zyx(self.tcp_pose[:3, :3])
            osd(f"TCP x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} "
                f"rpy=[{r:+.2f} {p:+.2f} {y:+.2f}]", 72)
