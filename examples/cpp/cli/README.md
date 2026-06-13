# C++ CLI Example

This is a minimal command-line Mighty SDK example. It has no OpenCV dependency.

It connects to the default SDK host list, starts VIO, prints status/VIO/pose
updates, then exits after a fixed duration.

## Build

From this directory:

```bash
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
./build/mighty_cli
```

Optional arguments:

```bash
./build/mighty_cli --host http://192.168.7.1 --seconds 30
```

Options:

- `--host URL`: use a specific device or proxy URL.
- `--seconds N`: run duration, default `10`.
- `--no-start`: connect and print stream events without sending `start_vio`.
- `--help`: print usage.
