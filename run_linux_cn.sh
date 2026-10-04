#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
MIRROR="https://pypi.tuna.tsinghua.edu.cn/simple"
export PIP_INDEX_URL="$MIRROR"
export PIP_DEFAULT_TIMEOUT=120
echo "LeatherCAD V0.3 - 使用清华 TUNA PyPI 国内镜像"
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -i "$MIRROR" --upgrade pip setuptools wheel
python -m pip install -i "$MIRROR" -r requirements.txt
python app.py
