# Python Examples

This folder contains Python examples for the Mighty protocol SDK.

## Preview

![Live VIO Dashboard Preview](./screen.png)

## Live VIO Dashboard

Main script: `mightyapp.py`

Support module: `uihelpers.py` (UI/layout/rendering + plotting helpers)

Features:
- Streams camera image.
- Shows VIO status.
- Plots IMU accel/gyro traces.
- Plots 3D pose trajectory.

## Synced Stereo Preview

Main script: `synced_stereo/main.py`

Features:
- Connects to two cameras at `http://192.168.7.1` and `http://192.168.8.1` by default.
- Aligns raw device timestamps into a shared synced timestamp base.
- Shows both camera previews and IMU accel/gyro traces side by side in an OpenCV window.
- Replays previously recorded synced stereo ROS1 bags with `--bag`.
- Button-controlled ROS1 bag recording to `~/synced_stereo_<timestamp>.bag`; install `opencv-python` and `rosbags` from `synced_stereo/requirements.txt`.

## Dependencies

Required:
- `python >= 3.9`
- `numpy`
- `matplotlib`

Optional:

Install:

```bash
pip install numpy matplotlib
```

Using `venv` (recommended), from the root of repository:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install numpy matplotlib
```

## Run

```bash
source .venv/bin/activate
cd examples/python
python3 mightyapp.py
```
