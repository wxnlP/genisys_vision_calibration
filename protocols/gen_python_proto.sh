#!/usr/bin/env bash
# Regenerate the Python headers (*_pb2.py / *_pb2.pyi) for protocols/proto/*.proto
# into protocols/.
#
# Run from anywhere:  ./protocols/gen_python_proto.sh
#
# Notes:
#  * protoc 33.5 emits gencode "Protobuf Python Version: 6.33.5", the same
#    generation used by the sibling statics_calibration project. It needs a
#    protobuf runtime >= 6.x -- the robo_aimrt conda env (protobuf 7.35.1) works;
#    the system python3 (protobuf 3.12.4) CANNOT import these files.
#  * protoc writes an *absolute* `import header_pb2`, which breaks
#    `from protocols.joint_state_pb2 import JointStateArray`. The sed step below
#    rewrites cross-proto imports to be relative so the modules resolve as a
#    package (this mirrors statics_calibration/protocols/pb/*_pb2.py).
set -euo pipefail

cd "$(dirname "$0")/.."   # project root

protoc -I protocols/proto \
       --python_out=protocols \
       --pyi_out=protocols \
       protocols/proto/*.proto

# absolute -> relative imports for generated cross-proto dependencies
sed -i -E 's/^import ([A-Za-z0-9_]+_pb2) as ([A-Za-z0-9_]+)$/from . import \1 as \2/' \
    protocols/*_pb2.py protocols/*_pb2.pyi

echo "regenerated:"
ls -1 protocols/*_pb2.py protocols/*_pb2.pyi
