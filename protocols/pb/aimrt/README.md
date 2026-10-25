# AimRT 基础 protobuf（vendor）

来源：AimRT v1.7.1（genisys 依赖中的 `build/_deps/aimrt-src`）。

| 文件 | proto package | 消息 |
|:---|:---|:---|
| `header.proto` | `aimrt.protocols.common` | `Header` |
| `quaternion.proto` | `aimrt.protocols.geometry` | `Quaternion` |
| `vector3.proto` | `aimrt.protocols.geometry` | `Vector3` |
| `joint_state.proto` | `aimrt.protocols.sensor` | `JointState` / `JointStateArray` |

这些文件来自 AimRT 上游，**不要在本工程里修改**；升级 AimRT 时整体替换。

`proto` 的 package 名和消息名必须保持不变，否则 AimRT 话题上的类型名会变化。
