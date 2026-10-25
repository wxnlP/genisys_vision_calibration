# protocols

消息定义按 **传输方式** 和 **来源** 组织：protobuf 在 `pb/`，ROS2 在 `ros2/`；每个来源（AimRT 上游、genisys）各自一个子目录。

```
protocols/
├── pb/
│   ├── aimrt/          # AimRT 上游基础 pb
│   ├── genisys/        # genisys 提供的 pb
│   └── _gen/           # protoc 生成物
├── ros2/
│   ├── genisys_msgs/   # ROS2 msg/srv 源码
│   ├── build/          # colcon 产物
│   ├── install/        # colcon 产物，固定前缀
│   ├── log/            # colcon 产物
│   └── rpc_stubs/      # AimRT Python RPC 桩
├── gen_python_proto.sh
├── build_genisys_msgs.sh
└── README.md
```

## 依赖

- `protoc`（本环境 33.5）；生成的模块需要 protobuf 7.x 运行时。
- ROS2 humble + colcon（构建 `genisys_msgs`）。

## protobuf

| 目录 | 内容 | 来源 |
|:---|:---|:---|
| `pb/aimrt/` | `Header` / `Quaternion` / `Vector3` / `JointState` | AimRT v1.7.1，vendor |
| `pb/genisys/` | `Pose` / `PoseStamped` | genisys `src/protocols/pb/genisys/` |
| `pb/_gen/` | `*_pb2.py` / `*_pb2.pyi` | `./gen_python_proto.sh` |

首次 clone 或修改 `.proto` 后：

```bash
./protocols/gen_python_proto.sh
```

然后在代码里通过统一出口导入：

```python
from protocols.pb import PoseStamped
```

`protocols/pb/__init__.py` 会 re-export `Header`、`Pose`、`PoseStamped`、`Quaternion`、`Vector3`、`JointState`、`JointStateArray`。

## ROS2

`ros2/genisys_msgs/` 是完整的 genisys IDL（`msg/` + `srv/`），但只保留 ROS2 构建，不包含 genisys 原 CMakeLists 里的 AimRT C++ RPC gencode。

```bash
./protocols/build_genisys_msgs.sh
source protocols/ros2/install/local_setup.bash

python -c "from genisys_msgs.msg import TaskContext; from genisys_msgs.srv import MoveJPose"
```

构建产物固定在 `protocols/ros2/install/`。

如果需要 AimRT Python RPC 桩，按需生成到 `ros2/rpc_stubs/`：

```bash
aimrt_py-gen-ros2-rpc \
  --pkg_name=genisys_msgs \
  --srv_file=protocols/ros2/genisys_msgs/srv/MoveJPose.srv \
  --output_path=protocols/ros2/rpc_stubs
```
