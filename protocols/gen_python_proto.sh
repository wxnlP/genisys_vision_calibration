#!/usr/bin/env bash
# 从 protocols/pb/{aimrt,genisys}/*.proto 生成 Python 模块到 protocols/pb/_gen。
set -euo pipefail

cd "$(dirname "$0")/.."

PROTO_ROOT=protocols/pb
GEN_DIR="$PROTO_ROOT/_gen"

if ! command -v protoc >/dev/null 2>&1; then
    echo "未找到 protoc，请先安装 protobuf 编译器。" >&2
    exit 1
fi

mkdir -p "$GEN_DIR"

protoc -I "$PROTO_ROOT/aimrt" \
       -I "$PROTO_ROOT/genisys" \
       --python_out="$GEN_DIR" \
       --pyi_out="$GEN_DIR" \
       "$PROTO_ROOT"/aimrt/*.proto \
       "$PROTO_ROOT"/genisys/*.proto

# 把跨文件的绝对 import 改成相对 import，便于按包导入
sed -i -E 's/^import ([A-Za-z0-9_]+_pb2) as ([A-Za-z0-9_]+)$/from . import \1 as \2/' \
    "$GEN_DIR"/*_pb2.py "$GEN_DIR"/*_pb2.pyi

touch "$GEN_DIR/__init__.py"

echo "已生成到 $GEN_DIR"
