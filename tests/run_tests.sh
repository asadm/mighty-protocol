#!/usr/bin/env bash
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="$HERE/bin"
APRILTAG_TEST_BIN_DIR="${MIGHTY_TEST_BIN_DIR:-${TMPDIR:-/tmp}/mighty-protocol-tests}"
mkdir -p "$APRILTAG_TEST_BIN_DIR"
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

echo "[build] g++ cpp_depth_test.cpp"
g++ -std=c++17 -pthread -I"$HERE/.." "$HERE/cpp_depth_test.cpp" -o "$BIN_DIR/cpp_depth_test"

echo "[build] g++ cpp_calibration_test.cpp"
g++ -std=c++17 -I"$HERE/.." "$HERE/cpp_calibration_test.cpp" -o "$BIN_DIR/cpp_calibration_test"

echo "[build] g++ cpp_apriltags_test.cpp"
g++ -std=c++17 -pthread -I"$HERE/.." "$HERE/cpp_apriltags_test.cpp" -o "$APRILTAG_TEST_BIN_DIR/cpp_apriltags_test"

echo "[test] cpp AprilTag importer"
"$APRILTAG_TEST_BIN_DIR/cpp_apriltags_test" "$HERE/fixtures/wpilib-2026-rebuilt-welded.json"

echo "[test] node AprilTag importer"
node "$HERE/node_apriltags.test.js"

echo "[test] python AprilTag importer"
python3 "$HERE/python_apriltags_test.py"

echo "[test] cpp sdk unit"
"$BIN_DIR/cpp_sdk_test"

echo "[test] cpp sdk integration"
"$BIN_DIR/cpp_sdk_integration_test"

echo "[test] cpp pose contract"
"$BIN_DIR/cpp_pose_contract_test"

echo "[test] cpp depth"
"$BIN_DIR/cpp_depth_test"

echo "[test] cpp calibration"
"$BIN_DIR/cpp_calibration_test"

echo "[test] node roundtrip"
node "$HERE/node_roundtrip.test.js"

echo "[test] node sdk"
node "$HERE/node_sdk.test.js"

echo "[test] node depth"
node "$HERE/node_depth.test.js"

echo "[test] node calibration"
node "$HERE/node_calibration.test.js"

echo "[test] node pose contract"
node "$HERE/node_pose_contract.test.js"

echo "[test] python consumer"
python3 "$HERE/python_consumer_test.py"

echo "[test] python sdk"
python3 "$HERE/python_sdk_test.py"

echo "[test] python sdk integration"
python3 "$HERE/python_sdk_integration_test.py"

echo "[test] python depth"
python3 "$HERE/python_depth_test.py"

echo "[test] python calibration"
python3 "$HERE/python_calibration_test.py"

echo "[test] python pose contract"
python3 "$HERE/python_pose_contract_test.py"

echo "[test] cross-client AprilTag parity"
python3 "$HERE/cross_client_apriltag_parity_test.py"

echo "[test] cross-client pose parity"
python3 "$HERE/cross_client_pose_parity_test.py"

echo "[test] docs/sdk examples conformance"
python3 "$HERE/docs_conformance_test.py"

# Metadata-only dropped image frames and SDK decode bypass.
echo "[test] image-drop protocol"
g++ -std=c++17 -I"$HERE/.." "$HERE/cpp_image_drop_test.cpp" -o "$BIN_DIR/cpp_image_drop_test"
"$BIN_DIR/cpp_image_drop_test"
node "$HERE/node_image_drop.test.js"
python3 "$HERE/python_image_drop_test.py"
