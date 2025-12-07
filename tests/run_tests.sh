#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="$HERE/bin"
mkdir -p "$BIN_DIR"

echo "[build] g++ cpp_roundtrip.cpp"
g++ -std=c++17 -I"$HERE/.." "$HERE/cpp_roundtrip.cpp" -o "$BIN_DIR/cpp_roundtrip"

echo "[test] node roundtrip"
node "$HERE/node_roundtrip.test.js"

echo "[test] python consumer"
python3 "$HERE/python_consumer_test.py"
