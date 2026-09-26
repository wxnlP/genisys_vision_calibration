from __future__ import annotations

from pathlib import Path

from .base import CameraDriver, CameraFrameError
from .orbbec import OrbbecCamera
from .realsense import RealsenseCamera

__all__ = ["CameraDriver", "CameraFrameError", "OrbbecCamera", "RealsenseCamera", "make_camera"]


def make_camera(cfg: dict) -> CameraDriver:
    """Create a camera driver from the camera section of the config."""
    cam_cfg  = cfg.get("camera", {})
    cam_type = cam_cfg.get("type", "").lower()
    w   = cam_cfg.get("color_width",  1280)
    h   = cam_cfg.get("color_height", 720)
    fps = cam_cfg.get("fps", 30)

    # 可选的内参/畸变文件目录：<项目根>/cfg/calibration/<camera.type>/
    _root     = Path(__file__).resolve().parent.parent.parent
    calib_dir = str(_root / "cfg" / "calibration" / cam_type)

    if "orbbec" in cam_type:
        return OrbbecCamera(w, h, fps, calib_dir=calib_dir)
    elif "realsense" in cam_type:
        return RealsenseCamera(w, h, fps, calib_dir=calib_dir)
    else:
        raise ValueError(
            f"Unsupported camera type: {cam_type!r}\n"
            f"Set camera.type in cfg/hand_eye_cfg.yaml to:\n"
            f"  orbbec_gemini305 | realsense_d435i | realsense_d405"
        )
