# u3vdb

A small USB3 Vision (U3V) terminal tool that talks to the device’s control interface using UVCP (USB3 Vision Control Protocol).

- Auto-discovers the U3V control interface (class 0xEF, subclass 0x05) and its bulk IN/OUT endpoints.
- Supports custom VID/PID via CLI.
- Provides interactive and one-shot command modes.

## Build (CMake)

Requirements:
- libusb-1.0 development files (Ubuntu/Debian: `sudo apt install libusb-1.0-0-dev`)
- CMake 3.10+
- A C++17 compiler

```sh
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
```

The binary `u3vdb` will be in the `build` directory.

Optional install:
```sh
sudo cmake --install .
```

## Usage

```text
Usage: ./u3vdb [options] [command]
Options:
  -c, --command <cmd>   Execute a single command then exit
  -i, --interactive     Force interactive mode (default if no command)
  -r, --reset           Reset terminal session before use
  -p, --password <pwd>  Password for unlocking terminal (or use TY_TERM_PASS)
      --vid <id>        USB vendor ID (e.g., 0x04b4)
      --pid <id>        USB product ID (e.g., 0x1003)
  -h, --help            Show this message
```

Examples:
- Interactive with password:
  ```sh
  ./u3vdb --password U3V
  ```
- One-shot command:
  ```sh
  ./u3vdb -p U3V -c "ls -l"
  ```
- Specify VID/PID explicitly (hex or decimal):
  ```sh
  ./u3vdb --vid 0x04b4 --pid 0x1003 -p U3V
  ```
