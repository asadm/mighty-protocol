# Web Example (Vanilla JS + Vite)

This example shows SDK usage in plain browser JS using npm modules:
- `dashboard/`: SDK/device/client usage
- `mapper/`: live WASM mapper with a Three.js view matching the Pangolin mapper style
- `loopclosure/`: loop-closure WASM usage
- `shared/`: UI rendering helpers and shared styling

## Preview

![Web SDK Dashboard Preview](./dashboard/screen.png)

Dependencies:
- `mighty-protocol` from local path (`file:../..`)
- `three` from npm
- Vite for dev server

## Run

From the `mighty-protocol/examples/web` directory:

```bash
npm install
npm run dev
```

Open the local URL printed by Vite (default `http://localhost:8090`).

Dashboard page:

```text
http://localhost:8090/dashboard/
```

Mapper page:

```text
http://localhost:8090/mapper/
```

Loop-closure page:

```text
http://localhost:8090/loopclosure/
```

For a local bag replay, start VIO separately:

```bash
./build_app/app /Users/asad/datasets/mighty-singleboard/calabazas-stairs4-imufixed.bag \
  main/polaris/config/rockchip1_ov9281_halfres.yaml \
  --vio --wait_for_web_client --fps=30 --start_from=0
```

## Notes

- The mapper page prefers `http://localhost:8084`, then `http://localhost:8080`, then the device default.
- If your device host differs, update `vite.config.js` proxy target or pass explicit hosts in code.
- Mapper WASM updates are keyframe replacements. Consume `update.frames` from
  `NativeMapperWasm.onMapUpdate()` or use `onMapFrameUpdate()`.
