# Mighty Protocol

Mighty Protocol is the open source SDK and wire protocol implementation for
Mighty Camera. It lets applications communicate with a Mighty visual-inertial
odometry device over USB Ethernet using HTTP streams and command requests.

This repository focuses on making the protocol easy to use from Python,
JavaScript, C++, and ROS 2.

## What You Can Build

- Stream VIO pose, velocity, IMU, image, and status events.
- Start and stop onboard VIO from your own application.
- Read device identity and configuration values.
- Build dashboards, robot adapters, logging tools, and calibration utilities.
- Decode the same protocol consistently across Python, JavaScript, and C++.

## Connection Model

When Mighty Camera is plugged into a host over USB, it appears as a USB Ethernet
device. The camera runs DHCP on that USB link, assigns the host an address, and
serves SDK traffic from:

```text
http://192.168.7.1
```

The SDK default host order is:

1. `http://192.168.7.1`
2. `http://localhost:8080`
3. `http://localhost:8084`

The localhost entries are useful for development proxies and replay tools.

## Install

Python from GitHub:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install "git+https://github.com/asadm/mighty-protocol.git"
```

JavaScript from GitHub:

```bash
npm install https://github.com/asadm/mighty-protocol
```

C++ is header-only for the protocol/client SDK:

```cpp
#include "cpp/mighty_sdk.h"
```

For a tiny C++ command-line example, see
[`examples/cpp/cli`](./examples/cpp/cli).

## Quick Start

Python:

```python
import time
from mighty_sdk import MightyClient, MightyWebDevice

device = MightyWebDevice()
client = MightyClient(device)

client.on_pose(lambda p: print("pose", p["position_m"], p["confidence"]))
client.on_status(lambda s: print("status", s["text"]))
client.on_error(lambda e: print("error", e["scope"], e["message"]))

client.connect()
print(client.start_vio())

try:
    while True:
        time.sleep(0.25)
except KeyboardInterrupt:
    client.disconnect()
```

JavaScript:

```js
import { MightyClient, MightyWebDevice } from "mighty-protocol";

const client = new MightyClient(new MightyWebDevice());

client.onPose((p) => console.log("pose", p.positionM, p.confidence));
client.onStatus((s) => console.log("status", s.text));
client.onError((e) => console.error(e.scope, e.message));

await client.connect();
console.log(await client.startVio());
```

C++:

```cpp
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "cpp/mighty_sdk.h"

int main() {
  using namespace mighty_protocol::sdk;

  auto device = std::make_shared<MightyWebDevice>();
  MightyClient client(device);

  client.on_pose([](const PoseFrame& p) {
    std::cout << "pose " << p.position_m[0] << " "
              << p.position_m[1] << " "
              << p.position_m[2] << " confidence="
              << p.confidence << std::endl;
  });

  client.connect();
  client.start_vio();
  std::this_thread::sleep_for(std::chrono::seconds(10));
  client.disconnect();
}
```

## Examples

- Python dashboard: [`examples/python`](./examples/python)
- Web dashboard: [`examples/web`](./examples/web)
- C++ dashboard: [`examples/cpp/dashboard`](./examples/cpp/dashboard)
- C++ CLI: [`examples/cpp/cli`](./examples/cpp/cli)
- ROS 2 publisher: [`examples/ros2`](./examples/ros2)

## Documentation

- Device docs: [`docs/index.mdx`](./docs/index.mdx)
- SDK introduction: [`docs/sdk/index.mdx`](./docs/sdk/index.mdx)
- Quick starts: [`docs/sdk/quickstart`](./docs/sdk/quickstart)
- ROS 2 quickstart: [`docs/sdk/quickstart/ros2.mdx`](./docs/sdk/quickstart/ros2.mdx)
- Recipes: [`docs/sdk/recipes.mdx`](./docs/sdk/recipes.mdx)
- Troubleshooting: [`docs/sdk/troubleshooting.mdx`](./docs/sdk/troubleshooting.mdx)
- Pose guide: [`docs/sdk/pose.mdx`](./docs/sdk/pose.mdx)
- API reference: [`docs/sdk/api-reference.mdx`](./docs/sdk/api-reference.mdx)
- Protocol spec: [`docs/sdk/protocol.mdx`](./docs/sdk/protocol.mdx)

## Test

Run the cross-language protocol and SDK tests:

```bash
./tests/run_tests.sh
```

The tests cover packet round-trips, SDK dispatch behavior, pose contract
compatibility, and docs/example conformance.

## License

Apache License 2.0. See [`LICENSE`](./LICENSE).
