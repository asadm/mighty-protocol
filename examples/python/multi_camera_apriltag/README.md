# Multi-Camera AprilTag Relocalization

This isolated example connects to two Mighty cameras over `mighty-protocol`,
shows both camera previews with AprilTag overlays, plots both VIO trajectories in
one 3D world, and displays AprilTag relocalization `EVNT` messages.

The default endpoints are:

- red camera: `http://192.168.7.1`
- blue camera: `http://192.168.8.1`

## Setup

Before running this example:

1. Open the Mighty web visualizer for each camera.
2. Enable **Relocalize using AprilTags** on both cameras.
3. Save AprilTag rows on both cameras. Ideally both cameras should use the same
   AprilTag relocalization config, because the demo expects both VIO instances
   to reset into the same tag-defined world frame.
4. Put each camera on a different IP/subnet, for example `192.168.7.1` and
   `192.168.8.1`, then run this example against those two endpoints.

## Install

```bash
python3 -m pip install -r mighty-web/mighty-protocol/examples/python/multi_camera_apriltag/requirements.txt
```

## Live Demo

```bash
python3 mighty-web/mighty-protocol/examples/python/multi_camera_apriltag/main.py
```

Start both VIO instances from the UI with **Start VIO**, or start immediately:

```bash
python3 mighty-web/mighty-protocol/examples/python/multi_camera_apriltag/main.py --start-vio
```

Override endpoints if needed:

```bash
python3 mighty-web/mighty-protocol/examples/python/multi_camera_apriltag/main.py \
  --red-host http://192.168.7.1 \
  --blue-host http://192.168.8.1
```

## Recording

Use the **Record** button, or start recording on launch:

```bash
python3 mighty-web/mighty-protocol/examples/python/multi_camera_apriltag/main.py \
  --record \
  --record-dir ~/bags
```

The bag includes compressed preview images, IMU, plotted poses, AprilTag VIZ
payloads, and relocalization events for both cameras.

## Replay

```bash
python3 mighty-web/mighty-protocol/examples/python/multi_camera_apriltag/main.py \
  --bag ~/bags/multi_camera_apriltag_YYYYMMDD-HHMMSS.bag
```

Use `--bag-rate 0` for as-fast-as-possible replay or `--bag-loop` to loop.
