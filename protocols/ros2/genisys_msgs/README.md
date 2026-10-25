# genisys_msgs（本工程本地构建版）

IDL 来源：genisys 仓库 `src/protocols/ros2/genisys_msgs/`，包含全部 `msg/` 与 `srv/`。

与 genisys 原包的差异：

- 去掉了 AimRT C++ RPC gencode（`add_ros2_aimrt_rpc_gencode_target_for_one_file`）；
  本工程用 Python，RPC 桩由 `aimrt_py-gen-ros2-rpc` 单独生成。
- `package.xml` / `CMakeLists.txt` 只保留 ROS2 构建所需依赖。

构建：

```bash
./protocols/build_genisys_msgs.sh
```

构建产物固定在 `protocols/ros2/install/`，运行时：

```bash
source /opt/ros/humble/setup.bash
source protocols/ros2/install/local_setup.bash
```

可选：生成 AimRT Python RPC 桩（按需对使用的 `.srv` 生成）：

```bash
aimrt_py-gen-ros2-rpc \
  --pkg_name=genisys_msgs \
  --srv_file=protocols/ros2/genisys_msgs/srv/MoveJPose.srv \
  --output_path=protocols/ros2/rpc_stubs
```

