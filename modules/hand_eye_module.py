import threading
import time
from pathlib import Path

import aimrt_py as aimrt
import cv2
import rerun as rr
import yaml

from modules.http_rpc import RpcClient
from modules.task_context_motion import TaskContextMotionClient
from algo.hand_eye import CalibMode, HandEyeCalibrator
from algo.transforms import quat_to_mat4, rotation_matrix_to_euler_zyx
from drivers.camera import make_camera

try:
    from protocols.pb import PoseStamped
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "protocols/pb/_gen 不存在，请先运行 ./protocols/gen_python_proto.sh"
    ) from exc

try:
    from genisys_msgs.msg import TaskContext
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "genisys_msgs 不可用；请先运行 ./protocols/build_genisys_msgs.sh "
        "并 source protocols/ros2/install/local_setup.bash"
    ) from exc


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
MAX_FRAME_FLUSH = 15
FEEDBACK_STALE_S = 0.5
FEEDBACK_WAIT_S = 5.0


def _pose_to_mat4(msg):
    """genisys PoseStamped → 4x4 齐次变换（base 系）。"""
    p, q = msg.pose.position, msg.pose.orientation
    return quat_to_mat4(p.x, p.y, p.z, q.x, q.y, q.z, q.w)


class HandEyeModule(aimrt.ModuleBase):
    def __init__(self):
        super().__init__()
        # AimRT 对象
        self.core = aimrt.CoreRef()
        self.logger = aimrt.LoggerRef()
        self.work_executor = aimrt.ExecutorRef()
        self.sub_flange_pose = aimrt.SubscriberRef()
        self.sub_task_context = aimrt.SubscriberRef()

        # 控制接口（Initialize 里按配置构造）
        self.rpc = None
        self.motion = None

        # 最新法兰位姿（4x4，base 系）与本机单调时间戳；首帧前为 None
        self.flange_pose = None       # T_flange2base：标定样本的 T_gripper2base
        self._pose_lock = threading.Lock()
        self._flange_time = 0.0

        self.running = False

        # 运行模式：normal / vision_only / motion_only
        self._mode = "normal"
        self._vision_found = False

        # 相机 / 标定器（Initialize 里构造，线程里打开）
        self._cam = None
        self._cam_fps = 30
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
        rpc_url = str(config.get("rpc_url", "http://127.0.0.1:50080"))
        rpc_timeout_s = float(config.get("rpc_timeout_s", 30.0))
        # 标定样本用法兰位姿；运动完成由 task_context 通知。
        sub_flange_pose_topic = str(
            config.get("sub_flange_pose_topic", "/geniarm/flange_pose"))
        sub_task_context_topic = str(
            config.get("sub_task_context_topic", "/geniarm/task_context"))
        self._mode = str(config.get("mode", "normal"))
        cam_cfg = config["camera"]
        aruco_cfg = config["calibration"]["aruco"]
        he_method = config["calibration"].get("hand_eye_method", "TSAI")
        motion_cfg = config.get("motion") or {}
        rr_cfg = config.get("rerun") or {}
        self._rr_enabled = bool(rr_cfg.get("enabled", True))
        self._rr_port = int(rr_cfg.get("grpc_port", 9876))
        self._rr_quality = int(rr_cfg.get("jpeg_quality", 80))
        self._rr_every = max(1, int(rr_cfg.get("log_every_n_frames", 1)))
        self._rr_save = str(rr_cfg.get("save_rrd", "") or "")
        self._cam_fps = int(cam_cfg.get("fps", 30))

        self.work_executor = self.core.GetExecutorManager().GetExecutor(work_executor_name)
        self.sub_flange_pose = self.core.GetChannelHandle().GetSubscriber(
            sub_flange_pose_topic)
        aimrt.Subscribe(self.sub_flange_pose, PoseStamped, self.flange_pose_callback)

        # genisys 运动 RPC 只回执受理结果，完成状态由 task_context 的终态决定。
        self.rpc = RpcClient(rpc_url, rpc_timeout_s)
        self.motion = TaskContextMotionClient(
            self.rpc,
            log_info=lambda msg: aimrt.info(self.logger, msg),
            log_warn=lambda msg: aimrt.warn(self.logger, msg),
            should_stop=lambda: not self.running,
            timeout_margin_s=float(motion_cfg.get("timeout_margin_s", 3.0)),
            min_timeout_s=float(motion_cfg.get("min_timeout_s", 5.0)),
            move_retry=int(motion_cfg.get("move_retry", 2)),
            retry_delay_s=float(motion_cfg.get("retry_delay_s", 0.5)),
        )
        self.sub_task_context = self.core.GetChannelHandle().GetSubscriber(
            sub_task_context_topic)
        aimrt.Subscribe(self.sub_task_context, TaskContext, self.motion.on_context)

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
                   f"poses={len(CALIB_POSES_XYZ)} rpc={rpc_url} "
                   f"flange_pose={sub_flange_pose_topic} "
                   f"task_context={sub_task_context_topic}")
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

    def flange_pose_callback(self, msg):
        """当前法兰位姿：标定用的 T_gripper2base。"""
        if not self.running:
            return
        T = _pose_to_mat4(msg)
        with self._pose_lock:
            self.flange_pose = T
            self._flange_time = time.monotonic()

    def _flange_pose_snapshot(self):
        with self._pose_lock:
            if self.flange_pose is None:
                return None
            return self.flange_pose, self._flange_time

    def _wait_feedback(self, timeout_s: float = FEEDBACK_WAIT_S) -> bool:
        """等 flange_pose 首帧，避免无反馈时每个点位都要等一次超时。"""
        deadline = time.monotonic() + timeout_s
        while self.running and time.monotonic() < deadline:
            with self._pose_lock:
                if self.flange_pose is not None:
                    return True
            time.sleep(0.05)

        aimrt.error(self.logger,
                    f"[feedback] no flange_pose within {timeout_s:.1f}s; "
                    f"check genisys is running and the topic name in cfg")
        return False

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

        if self._mode == "normal" and not self._wait_feedback():
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
        """motion_only：只走完预设点位，不开相机、不等标记、不采样。"""
        total = len(CALIB_POSES_XYZ)
        ok = skip = 0
        for idx, (x, y, z, roll, pitch, yaw) in enumerate(CALIB_POSES_XYZ):
            if not self.running:
                break
            label = f"pose {idx + 1}/{total}"
            aimrt.info(self.logger,
                       f"[{label}] x={x:.3f} y={y:.3f} z={z:.3f} "
                       f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})")
            out = self.motion.move_to_pose(x, y, z, roll, pitch, yaw, label=label)
            if not out.ok:
                aimrt.warn(self.logger, f"[{label}] move failed, skip: {out.reason}")
                skip += 1
                continue
            ok += 1
            snapshot = self._flange_pose_snapshot()
            if snapshot is None:
                aimrt.warn(self.logger, f"[{label}] no flange_pose feedback")
                continue
            T = snapshot[0]
            t = T[:3, 3]
            r, p, yy = rotation_matrix_to_euler_zyx(T[:3, :3])
            aimrt.info(self.logger,
                       f"[{label}] reached flange x={t[0]:+.4f} y={t[1]:+.4f} z={t[2]:+.4f} "
                       f"rpy=({r:+.3f},{p:+.3f},{yy:+.3f}) task={out.task_id}")
        aimrt.info(self.logger, f"[motion] done ok={ok} skip={skip} total={total}")

    # ---------- 自动遍历状态机 ----------

    def _start_next_pose(self) -> bool:
        """发下一条运动指令并等待到位（阻塞）。返回 True 表示全部点位走完。"""
        total = len(CALIB_POSES_XYZ)
        while self._auto["idx"] < total:
            if not self.running:
                break
            idx = self._auto["idx"]
            x, y, z, roll, pitch, yaw = CALIB_POSES_XYZ[idx]
            label = f"pose {idx + 1}/{total}"
            aimrt.info(self.logger,
                       f"[{label}] x={x:.3f} y={y:.3f} z={z:.3f} "
                       f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})")

            out = self.motion.move_to_pose(x, y, z, roll, pitch, yaw, label=label)
            if out.ok:
                self._auto["pose_idx"] = idx
                self._auto["phase"] = "searching"
                self._auto["timeout_at"] = time.monotonic() + AUTO_MARKER_TIMEOUT_S
                self._auto["stable_frames"] = 0
                # 运动期间缓存的相机帧已经过时，按运动时长丢弃（上限见 MAX_FRAME_FLUSH）
                flush_count = min(
                    int(out.duration_ms / 1000.0 * self._cam_fps) + FRAME_FLUSH_AFTER_MOVE,
                    MAX_FRAME_FLUSH)
                self._flush_frames(flush_count)
                return False

            aimrt.warn(self.logger, f"[{label}] move failed, skip: {out.reason}")
            self._auto["idx"] += 1

        if not self.running:
            return False
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

    def _flush_frames(self, count: int = FRAME_FLUSH_AFTER_MOVE):
        """丢弃运动期间积压的旧帧。"""
        for _ in range(max(0, int(count))):
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
        snapshot = self._flange_pose_snapshot()
        if snapshot is None:
            aimrt.warn(self.logger, f"[{source}] no flange pose yet, sample dropped")
            return False
        T_g2b, stamp = snapshot
        if time.monotonic() - stamp > FEEDBACK_STALE_S:
            aimrt.warn(self.logger, f"[{source}] flange pose is stale, sample dropped")
            return False

        c = self._calibrator
        m = marker.T_marker2cam[:3, 3]
        t = T_g2b[:3, 3]
        c.add_sample(T_g2b, marker.T_marker2cam)
        aimrt.info(self.logger,
                   f"[sample {c.n_samples}] {source} "
                   f"marker=({m[0]:+.3f},{m[1]:+.3f},{m[2]:.3f}) "
                   f"flange=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})")
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
                       f"[result] n={c.n_samples} frame=flange "
                       f"t_cam2flange=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})")
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
            osd(f"[ID={marker.id}] z={marker.T_marker2cam[2, 3]:.3f}m  samples:{n}",
                28, (80, 220, 80))
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
                          f"{auto['stable_frames']}/{AUTO_MARKER_STABLE_FRAMES} "
                          f"wait {remain:.1f}s")
            osd(f"AUTO: {status}", 50, (180, 180, 60))

            filled = min(n, 15) * (400 // 15)
            cv2.rectangle(vis, (10, 110), (10 + filled, 122), (0, 200, 100), -1)
            cv2.rectangle(vis, (10, 110), (410, 122), (160, 160, 160), 1)
            cv2.putText(vis, f"{n}/15", (10, 137), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (200, 200, 200), 1)

        flange_snapshot = self._flange_pose_snapshot()
        if flange_snapshot is not None:
            T = flange_snapshot[0]
            t = T[:3, 3]
            r, p, y = rotation_matrix_to_euler_zyx(T[:3, :3])
            osd(f"FLANGE x={t[0]:+.3f} y={t[1]:+.3f} z={t[2]:+.3f} "
                f"rpy=[{r:+.2f} {p:+.2f} {y:+.2f}]", 72)
