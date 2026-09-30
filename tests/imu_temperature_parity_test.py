"""Run C++ (including fast-math), JS and Python optional IMU temperature checks."""
import pathlib
import struct
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "python"))
import mighty_protocol as mp
from mighty_sdk import MightyClient

with tempfile.TemporaryDirectory(prefix="mighty-imu-temperature-") as directory:
    folder = pathlib.Path(directory)
    binary = folder / "cpp-test"
    subprocess.run(["g++", "-std=c++17", "-O3", "-ffast-math",
                    str(HERE / "cpp_imu_temperature_test.cpp"), "-o", str(binary)], check=True)
    subprocess.run([str(binary), directory], check=True)
    subprocess.run(["node", str(HERE / "node_imu_temperature.test.js"), directory], check=True)
    expected = [dict(timestamp_ns=1000000000+i*1250000, ax=float(i),
                     ay=2., az=3., gx=4., gy=5., gz=6.) for i in range(5)]
    modern = [dict(s) for s in expected]
    for index, temperature in [(0, -12.5), (1, 0.), (3, 46.5)]:
        modern[index]["temperature_c"] = temperature
    for name in ["legacy", "modern", "truncated", "unknown", "bad-count"]:
        payload = (folder / (name + ".bin")).read_bytes()
        assert mp.decode_imu_payload(payload) == (modern if name == "modern" else expected)
        # Legacy timestamp/motion layout is unchanged, including every sample.
        count, = struct.unpack_from(">I", payload)
        assert count == len(expected)
        for index in range(count):
            assert struct.unpack_from(">Q6d", payload, 4 + index * 56) == tuple(expected[index].values())
    class Device:
        def connect(self, *args): pass
        def disconnect(self): pass
    client = MightyClient(Device())
    received = []
    client.on_imu(lambda batch: received.append(batch["samples"]))
    client._handle_frame({"type": "IMU ", "payload": (folder / "modern.bin").read_bytes()})
    assert received == [modern]
    try:
        mp.decode_imu_payload(b"\xff" * 4)
        raise AssertionError("invalid count accepted")
    except ValueError:
        pass
print("IMU temperature: C++ fast-math, JS and Python/SDK compatibility passed")
