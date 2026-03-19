# Mighty SDK

SDK and protocol helpers to interact with **Mighty Camera** devices ([mightycamera.com](https://mightycamera.com)).

This repository provides:
- High-level SDK clients for **Python**, **JavaScript (ESM)**, and **C++**.
- A transport abstraction (`MightyWebDevice`) for HTTP streaming + command RPC.
- Low-level protocol encode/decode helpers (see [PROTOCOL.md](./PROTOCOL.md)).

## Quick Start

### 1) Python

#### Setup

From `mighty-protocol/`:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
# Optional for image preview examples:
pip install numpy opencv-python

# Use repo-local SDK modules
export PYTHONPATH="$(pwd)/python:${PYTHONPATH}"
```

#### Subscribe to pose and image

```python
import time
from mighty_sdk import MightyWebDevice, MightyClient

# Uses built-in host scan order:
# http://localhost:8080, http://localhost:8084, http://192.168.7.1:80, http://192.168.7.1:8080
device = MightyWebDevice()
client = MightyClient(device, auto_reconnect=True)


def on_pose(p):
    x, y, z = p["position"]
    print(f"pose xyz: {x:.3f}, {y:.3f}, {z:.3f}  conf={p['confidence']:.3f}")


def on_image(img):
    if img["kind"] == "raw":
        print(
            f"raw frame: {img['width']}x{img['height']} "
            f"ch={img['channel']} fmt={img['format']} bytes={len(img['data'])}"
        )

        # Preview path (left unimplemented intentionally):
        # - img["data"] is the raw byte plane
        # - map by img["format"] and reshape using width/height
        # - render using OpenCV / matplotlib / your GUI toolkit


client.on_pose(on_pose)
client.on_image(on_image)
client.on_error(lambda e: print("error:", e))

client.connect()

# Optional: tell device to start VIO
res = client.start_vio()
print("start_vio:", res)

try:
    while True:
        time.sleep(0.25)
except KeyboardInterrupt:
    client.disconnect()
```

See full Python GUI example:
- [`examples/python/mightyapp.py`](./examples/python/mightyapp.py)

### 2) JavaScript (web / Node ESM)

#### Install

In your downstream web app:

```bash
npm install https://github.com/asadm/mighty-protocol
```

#### Subscribe to pose and image

```js
import { MightyWebDevice, MightyClient, decodeRawToRgb } from "mighty-protocol";

const device = new MightyWebDevice();
const client = new MightyClient(device, {
  autoReconnect: true,
  reconnectDelayMs: 500,
});
const canvas = document.querySelector("#cam");
const ctx = canvas.getContext("2d");

client.onPose((pose) => {
  const [x, y, z] = pose.position;
  console.log("pose xyz:", x.toFixed(3), y.toFixed(3), z.toFixed(3), "conf", pose.confidence);
});

client.onImage((img) => {
  console.log("raw/gray8 frame", img.width, img.height, img.channel, img.data.length);
  const decoded = decodeRawToRgb(img);
  decoded && ctx.putImageData(new ImageData(decoded.rgba, decoded.width, decoded.height), 0, 0);
});

client.onError((e) => console.error("error", e.scope, e.code, e.message));

await client.connect();
await client.startVio();
```

See full web dashboard example:
- [`examples/web/main.js`](./examples/web/main.js)

### 3) C++

C++ SDK is header-only; include `cpp/mighty_sdk.h` and `cpp/mighty_protocol.h`.

```cpp
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "cpp/mighty_sdk.h"

int main() {
  using namespace mighty_protocol::sdk;

  auto device = std::make_shared<MightyWebDevice>();  // uses default host scan list
  MightyClient client(device);

  auto pose_sub = client.on_pose([](const PoseFrame& p) {
    std::cout << "pose xyz: "
              << p.position[0] << ", "
              << p.position[1] << ", "
              << p.position[2]
              << " conf=" << p.confidence << "\n";
  });

  auto img_sub = client.on_image([](const ImageFrame& f) {
    if (f.kind == ImageFrame::Kind::kRaw) {
      std::cout << "raw frame: "
                << f.left.width << "x" << f.left.height
                << " ch=" << f.left.channel
                << " fmt=" << static_cast<int>(f.left.format)
                << " bytes=" << f.left.data.size() << "\n";

      // Preview path (left unimplemented intentionally):
      // f.left.data contains raw bytes; convert by format and render in your app/UI.
    }
  });

  client.on_error([](const MightyErrorEvent& e) {
    std::cerr << "error " << e.scope << ":" << e.code << " " << e.message << "\n";
  });

  client.connect();
  const auto start = client.start_vio();
  std::cout << "start_vio ok=" << start.ok << " msg=" << start.message << "\n";

  std::this_thread::sleep_for(std::chrono::seconds(30));
  client.disconnect();
  return 0;
}
```

## Commands and Config

All SDKs expose command/config helpers:
- `start_vio`, `stop_vio`
- generic `command(name, payload)`
- `config_get` / `config_set` (or camelCase equivalents by language)

Typical command flow:
- SDK builds `CMD`
- device responds with `CRES`
- config uses `CFGQ/CFGR` payloads wrapped in command `name="config"`

## Host Discovery Defaults

`MightyWebDevice()` with no args tries hosts in this order:
1. `http://localhost:8080`
2. `http://localhost:8084`
3. `http://192.168.7.1:80`
4. `http://192.168.7.1:8080`

You can override with `base_url` / `base_urls` (language-specific naming).

## Examples

- Python GUI example: [`examples/python/`](./examples/python/)
- Web dashboard example: [`examples/web/`](./examples/web/)
- Integration tests: [`tests/`](./tests/)

## Protocol Specification

Low-level packet framing and payload layouts are documented in:
- [PROTOCOL.md](./PROTOCOL.md)

## Running Tests

From `mighty-protocol/`:

```bash
./tests/run_tests.sh
```

This runs:
- C++ roundtrip and SDK tests
- Node/JS roundtrip and SDK tests
- Python protocol and SDK tests
