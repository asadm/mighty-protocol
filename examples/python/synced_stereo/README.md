# Synced Stereo Preview

This example connects to two Mighty cameras at different USB IP ranges, aligns
their device timestamp bases, and displays camera previews plus IMU traces side
by side in an OpenCV window.

Default endpoints:

- Left: `http://192.168.7.1`
- Right: `http://192.168.8.1`

Hardware setup:

1. Mount two Mighty cameras on a rigid mount with a fixed baseline. For testing,
   you can use `mighty-web/mighty-protocol/examples/python/synced_stereo/stereo-mount.stl`.
2. Connect the FSYNC pins of both cameras together.
3. Connect both FSYNC pins to the PWM pin from one camera, for example: both
   cameras' FSYNC pins connected to camera 1's PWM pin.
4. Open the web dashboard at <https://mightycamera.com/viz> and change one
   camera's IP to `192.168.8.1`. Leave the other camera at `192.168.7.1`.
5. Enable SYNC mode on both cameras.
6. Once mounted, wired, addressed, and in SYNC mode, run this app.

Run from the repository root:

```bash
pip install -r mighty-web/mighty-protocol/examples/python/synced_stereo/requirements.txt
python3 mighty-web/mighty-protocol/examples/python/synced_stereo/main.py
```

Run without depth:

```bash
python3 mighty-web/mighty-protocol/examples/python/synced_stereo/main.py \
  --left-host http://192.168.7.1 \
  --right-host http://192.168.8.1
```

Run with rectified SGBM metric depth:

```bash
python3 mighty-web/mighty-protocol/examples/python/synced_stereo/main.py \
  --left-host http://192.168.7.1 \
  --right-host http://192.168.8.1 \
  --depth ~/path/to/stereo-calibration-camchain-imucam.yaml \
  --depth-scale 0.5 \
  --depth-min-m 0.2 \
  --depth-max-m 5.0
```

Depth mode adds an SGBM disparity band to the main OpenCV window. It updates on
each newly rendered synced stereo pair. The depth colors are metric, not
per-frame relative: `--depth-min-m` maps to the hottest color and
`--depth-max-m` maps to the coolest color.

File layout:

- `main.py`: docs-friendly entry point and app lifecycle
- `depth.py`: Kalibr double-sphere rectification plus SGBM disparity
- `viewer.py`: OpenCV window, paired preview, IMU plots, record button
- `recording.py`: ROS1 bag writer and recording controller
- `state.py`: camera state, clock offsets, and synced image-pair selection
- `stream.py`: Mighty SDK callbacks and clock sync lifecycle
- `utils.py`: small timestamp, median, and raw-frame helpers

The app first samples each camera's `clock` command to estimate
`device steady_ns -> host monotonic_ns`. It then optionally refines the second
camera's offset using image pairs whose host arrival times are close, assuming
the shared SYNC shutter pin makes those frames simultaneous.

The preview renders paired images, not just the latest raw frame from each
camera. A pair is selected from the recent frame buffers when the two synced
timestamps are within `--render-pair-max-delta-ms` (default `8.0`). The overlay
shows the raw timestamp, synced timestamp, rendered pair delta, and rendered pair
age.

Stereo calibration workflow:

1. Run this app without `--depth`.
2. Open <https://mightycamera.com/calib> in full screen and point both cameras at
   the AprilGrid.
3. Click `Start Rec` in this app.
4. Move the rigid stereo rig through the needed calibration motions while keeping
   the cameras pointed at the grid.
5. Click `Stop Rec` when done.
6. On the `/calib` page, measure and enter the correct AprilGrid width first.
7. Open **Advanced** and set **Camera topics** to
   `/cam0/image_raw/compressed /cam1/image_raw/compressed`.
8. Manually upload the recorded bag from this app as a custom ROS bag.
9. Start calibration from that custom ROS bag. It should take about 15 minutes.
10. Download the report and find the `-camchain-imucam.yaml` file.
11. Run this app again with `--depth /path/to/*-camchain-imucam.yaml` to enable
    rectified SGBM metric depth.

Useful options:

```bash
python3 mighty-web/mighty-protocol/examples/python/synced_stereo/main.py \
  --left-host http://192.168.7.1 \
  --right-host http://192.168.8.1 \
  --start-vio
```

Record a ROS1 bag while previewing by clicking `Start Rec` in the OpenCV
window. Click `Stop Rec` to close the bag. Bags are saved in `~/` by default as
`synced_stereo_<timestamp>.bag`.

```bash
python3 mighty-web/mighty-protocol/examples/python/synced_stereo/main.py
```

Use `--record-dir /path/to/dir` to change the default save directory. Use
`--record-out /path/to/file.bag` when you want the next recording to use a
specific filename. `--record` still works as a convenience flag to start
recording immediately on launch.

Default recording topics:

- `/cam0/image_raw/compressed` (`sensor_msgs/CompressedImage`) for the left camera
- `/cam1/image_raw/compressed` (`sensor_msgs/CompressedImage`) for the right camera
- `/imu0` (`sensor_msgs/Imu`) for the left IMU
- `/imu1` (`sensor_msgs/Imu`) for the right IMU

Images are recorded only as matched stereo pairs. Each left/right image pair is
written with the same ROS header timestamp and bag timestamp. IMU samples are
written with their synced timestamps in the same clock base.

Use `--no-image-refine` to rely only on clock-command alignment.

Enable the optional rectified stereo depth view with a Kalibr camchain YAML:

```bash
python3 mighty-web/mighty-protocol/examples/python/synced_stereo/main.py \
  --depth /Users/asad/Documents/kalibr-rerun-161436/synced_stereo_20260626-161436-camchain-imucam.yaml
```

Depth mode uses the same rendered synced stereo pair as the raw preview. It
rectifies the double-sphere cameras from the Kalibr YAML, converts SGBM
disparity to metric depth using `Z = focal_px * baseline_m / disparity_px`, then
shows rectified left/right frames plus SGBM metric depth in the main window. Use
`--depth-scale` to trade detail for speed and `--depth-min-m` / `--depth-max-m`
to tune the fixed color range.

Dependencies:

```bash
pip install numpy opencv-python PyYAML rosbags
```
