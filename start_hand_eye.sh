#!/bin/bash

SCRIPT_DIR="$( cd $( dirname ${BASH_SOURCE[0]} ) && pwd )"
PYTHON_BIN="${HOME}/miniconda3/envs/robo_aimrt/bin/python"

ROS2_PREFIX="${ROS2_PREFIX:-/opt/ros/${ROS_DISTRO:-humble}}"
if [ -f "$ROS2_PREFIX/setup.bash" ]; then
    source "$ROS2_PREFIX/setup.bash"
fi

GENISYS_MSGS_SETUP="${SCRIPT_DIR}/protocols/ros2/install/local_setup.bash"
if [ -f "$GENISYS_MSGS_SETUP" ]; then
    source "$GENISYS_MSGS_SETUP"
fi

export AIMRT_PLUGIN_DIR="$(${PYTHON_BIN} -c 'import aimrt_py, pathlib; print(pathlib.Path(aimrt_py.__file__).resolve().parent)')"
export AIMRT_SH_DIR="${SCRIPT_DIR}"

${PYTHON_BIN} "${SCRIPT_DIR}/main.py" \
    --cfg_file_path="${SCRIPT_DIR}/cfg/hand_eye_cfg.yaml"
