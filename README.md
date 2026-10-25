# genisys_vision_calibration

genisys 机械臂手眼标定（Eye-in-Hand / TSAI），标定 **法兰与相机之间** 的固定变换。

## 标定目标

相机固定在法兰（末端）上。程序输出的 `T_result` 是：

- **相机坐标系 → 法兰坐标系** 的变换，即 `T_cam2flange`（`T_flange←cam`）；
- `p_flange = T_result @ p_cam`；
- 需要反向（法兰 → 相机）时取 `np.linalg.inv(T_result)`。

采样流程：

- 订阅 `/geniarm/flange_pose`，作为每个样本的 `T_gripper2base`；
- 用 `MoveJPose` 遍历预设点位，订阅 `/geniarm/task_context` 判断运动完成；
- ArUco 提供 `T_marker2cam`；
- 多组 `(T_gripper2base, T_marker2cam)` 求解，结果保存到
  `cfg/calibration/<camera.type>/hand_eye.npz`。

注意：采样使用的是法兰位姿 `/geniarm/flange_pose`，不是 TCP；因此标定结果是相机与法兰之间的变换。

## 环境依赖与启动前准备

### ROS2 / ROS Message

程序使用 `genisys_msgs/msg/TaskContext`（ROS2 消息）判断运动完成，依赖 ROS2 类型支持。
启动前先让 ROS2 和本工程构建的 `genisys_msgs` 可见：

```bash
source /opt/ros/humble/setup.bash
source protocols/ros2/install/local_setup.bash
```

`start_hand_eye.sh` 会自动执行这两步；手动运行 `main.py` 时必须先 source。

首次使用前构建 `genisys_msgs`：

```bash
./protocols/build_genisys_msgs.sh
```

协议目录、IDL 和 ROS2 构建细节见 [protocols/README.md](protocols/README.md)。

### Python / 运行环境

- Python 依赖：`aimrt_py`、`numpy`、OpenCV（含 aruco 模块）、`rerun-sdk`、`PyYAML`、`requests`；
- 首次使用前生成 protobuf Python 模块：

  ```bash
  ./protocols/gen_python_proto.sh
  ```

- genisys 已启动：HTTP RPC `127.0.0.1:50080`，zenoh 话题`/geniarm/flange_pose`、`/geniarm/task_context`。

生成/构建脚本和目录说明见 [protocols/README.md](protocols/README.md)。

## 启动

```bash
./start_hand_eye.sh
```

脚本使用 `cfg/hand_eye_cfg.yaml`，并自动设置 `AIMRT_PLUGIN_DIR`、source ROS2 与
`genisys_msgs`。如果 conda 环境不在默认路径，修改脚本里的 `PYTHON_BIN`。

手动启动：

```bash
source /opt/ros/humble/setup.bash
source protocols/ros2/install/local_setup.bash
export AIMRT_PLUGIN_DIR="$("${HOME}/miniconda3/envs/robo_aimrt/bin/python" -c \
  'import aimrt_py, pathlib; print(pathlib.Path(aimrt_py.__file__).resolve().parent)')"
"${HOME}/miniconda3/envs/robo_aimrt/bin/python" main.py \
  --cfg_file_path=cfg/hand_eye_cfg.yaml
```

运行模式：

| mode | 行为 |
|:---|:---|
| `normal` | 走位 + ArUco 识别 + 采样 + 求解保存 |
| `vision_only` | 只开相机识别，不运动、不求解 |
| `motion_only` | 只走预设点位，不采样、不求解 |

## 配置参数

配置文件：`cfg/hand_eye_cfg.yaml`。

| 参数 | 含义 | 当前/示例 |
|:---|:---|:---|
| `rpc_url` | genisys 运动 RPC（net_plugin HTTP） | `http://127.0.0.1:50080` |
| `rpc_timeout_s` | HTTP 请求超时，秒 | `30` |
| `sub_flange_pose_topic` | 法兰位姿话题，作为 `T_gripper2base` | `/geniarm/flange_pose` |
| `sub_task_context_topic` | 运动完成状态话题 | `/geniarm/task_context` |
| `motion.timeout_margin_s` | 等待超时 = `duration_ms/1000 + margin` | `3.0` |
| `motion.min_timeout_s` | 单次运动最小等待时间，秒 | `5.0` |
| `motion.move_retry` | 控制器忙/拒绝时的重试次数 | `2` |
| `motion.retry_delay_s` | 拒绝后重试前的等待，秒 | `0.5` |
| `mode` | 运行模式：`normal` / `vision_only` / `motion_only` | `motion_only` |
| `camera.type` | 相机驱动类型 | `orbbec_gemini305` |
| `camera.color_width` / `color_height` | 彩色图分辨率 | `1280 × 720` |
| `camera.fps` | 采集帧率 | `30` |
| `calibration.aruco.marker_length_m` | ArUco 黑框外沿边长，米 | `0.1` |
| `calibration.aruco.dict_id` | OpenCV ArUco 字典 ID | `0`（`DICT_4X4_50`） |
| `calibration.aruco.target_marker_id` | 只检测该 ID；不填/null 则取最近的标记 | `0` |
| `calibration.hand_eye_method` | OpenCV 手眼求解方法 | `TSAI` |
| `rerun.enabled` | 是否启用 rerun 可视化 | `true` |
| `rerun.grpc_port` | rerun gRPC 端口 | `9876` |
| `rerun.jpeg_quality` | rerun 图像 JPEG 质量 | `80` |
| `rerun.log_every_n_frames` | 每 N 帧记录一次 | `1` |
| `rerun.save_rrd` | 保存 `.rrd` 的路径；空则不保存 | `""` |
