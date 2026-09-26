#!/bin/bash

SCRIPT_DIR="$( cd $( dirname ${BASH_SOURCE[0]} ) && pwd )"
PYTHON_BIN="${HOME}/miniconda3/envs/robo_aimrt/bin/python"

export AIMRT_PLUGIN_DIR="$(${PYTHON_BIN} -c 'import aimrt_py, pathlib; print(pathlib.Path(aimrt_py.__file__).resolve().parent)')"
export AIMRT_SH_DIR="${SCRIPT_DIR}"

${PYTHON_BIN} "${SCRIPT_DIR}/main.py" \
    --cfg_file_path="${SCRIPT_DIR}/cfg/hand_eye_cfg.yaml"
