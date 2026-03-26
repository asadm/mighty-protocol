#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="$HERE/bin"
mkdir -p "$BIN_DIR"

echo "[build] g++ cpp_roundtrip.cpp"
g++ -std=c++17 -I"$HERE/.." "$HERE/cpp_roundtrip.cpp" -o "$BIN_DIR/cpp_roundtrip"

echo "[build] g++ cpp_sdk_test.cpp"
g++ -std=c++17 -pthread -I"$HERE/.." "$HERE/cpp_sdk_test.cpp" -o "$BIN_DIR/cpp_sdk_test"

echo "[build] g++ cpp_sdk_integration_test.cpp"
g++ -std=c++17 -pthread -I"$HERE/.." "$HERE/cpp_sdk_integration_test.cpp" -o "$BIN_DIR/cpp_sdk_integration_test"

echo "[build] g++ cpp_pose_contract_test.cpp"
g++ -std=c++17 -I"$HERE/.." "$HERE/cpp_pose_contract_test.cpp" -o "$BIN_DIR/cpp_pose_contract_test"

echo "[build] g++ cpp_pose_viz_map.cpp"
g++ -std=c++17 -I"$HERE/.." "$HERE/cpp_pose_viz_map.cpp" -o "$BIN_DIR/cpp_pose_viz_map"

echo "[test] cpp sdk unit"
"$BIN_DIR/cpp_sdk_test"

echo "[test] cpp sdk integration"
"$BIN_DIR/cpp_sdk_integration_test"

echo "[test] cpp pose contract"
"$BIN_DIR/cpp_pose_contract_test"

echo "[test] node roundtrip"
node "$HERE/node_roundtrip.test.js"

echo "[test] node sdk"
node "$HERE/node_sdk.test.js"

echo "[test] node pose contract"
node "$HERE/node_pose_contract.test.js"

echo "[test] python consumer"
python3 "$HERE/python_consumer_test.py"

echo "[test] python sdk"
python3 "$HERE/python_sdk_test.py"

echo "[test] python sdk integration"
python3 "$HERE/python_sdk_integration_test.py"

echo "[test] python pose contract"
python3 "$HERE/python_pose_contract_test.py"

echo "[test] cross-client pose parity"
python3 "$HERE/cross_client_pose_parity_test.py"

echo "[test] docs/examples conformance"
python3 "$HERE/docs_conformance_test.py"
