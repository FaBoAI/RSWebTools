#!/usr/bin/env bash
# RobStride QDD コンフィギュレータを起動する。
set -euo pipefail
cd "$(dirname "$0")"

if [ ! -d .venv ]; then
  echo "初回セットアップ: 仮想環境を作成します..."
  python3 -m venv .venv
  ./.venv/bin/pip install -q --upgrade pip
  ./.venv/bin/pip install -q -r requirements.txt
fi

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8765}"
echo "http://${HOST}:${PORT} を開いてください"
exec ./.venv/bin/python -m uvicorn backend.server:app --host "$HOST" --port "$PORT" "$@"
