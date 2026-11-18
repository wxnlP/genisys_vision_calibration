# genisys protobuf

来源：genisys 仓库 `src/protocols/pb/genisys/`。

| 文件 | proto package | 消息 |
|:---|:---|:---|
| `pose.proto` | `genisys.protocols` | `Pose` / `PoseStamped` |
| `pb_proto.proto` | `genisys.pb_proto` | `change_me`（上游占位消息） |

`pose.proto` 依赖 `header.proto`、`quaternion.proto`、`vector3.proto`，
生成时通过 `protocols/pb/aimrt` 的 include 路径解析。

同步方式：从 genisys 复制全部 `.proto`，并同步协议版本。
`proto` package 名与字段布局必须与 genisys 保持一致。
