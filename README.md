# dw3k-daemon

## Overview

`dw3k-daemon` is a long-running Linux service dedicated to communicating with a DWM3000 UWB Module from Qorvo over SPI.
It handles SPI bring-up, continuous health monitoring, and exposes a JSON/Unix
domain socket API so any process (Python, shell scripts, etc.) can drive the
chip without reinitializing it for each request.

Key capabilities:

- Controlled chip power state (`chip_enable` / `chip_disable`) so the radio is
  only energized when needed.
- Configurable PAN ID, local short address (`node_id`), antenna delay, socket
  path, and monitor interval via `/etc/dw3k-daemon.conf`.
- Health polling thread that keeps track of DW3K readiness and last
  measurement statistics.
- DS-TWR ranging requests with per-call role/peer selection and JSON results.

## Building & Installing

```bash
cmake -S . -B build
cmake --build build -j
sudo cmake --install build
```

By default the binary is installed to `${CMAKE_INSTALL_SBINDIR}` (typically
`/usr/local/sbin`) and the configuration file to
`${CMAKE_INSTALL_SYSCONFDIR}/dw3k-daemon.conf` (typically `/usr/local/etc` or
`/etc`). Deployment scripts can adjust those paths via standard CMake
`-DCMAKE_INSTALL_*` variables.

## Configuration File

- Path: `/etc/dw3k-daemon.conf` by default.
- Created automatically during installation and again at runtime if missing.
- Values accept decimal or hexadecimal notation (prefix with `0x`).

Supported keys:

| Key                   | Description                                            |
| --------------------- | ------------------------------------------------------ |
| `pan_id`              | 16-bit PAN shared across the UWB network.              |
| `node_id`             | 16-bit short address assigned to this SBC's DW3K chip. |
| `antenna_delay`       | TX/RX antenna delay in DWT units.                      |
| `monitor_interval_ms` | Health monitor polling interval (ms).                  |
| `socket_path`         | Unix domain socket exposed to clients.                 |

The daemon rewrites nothing inside the config file; edit it manually and
restart the service (or send runtime config commands) to apply changes.

## Runtime

Start in the foreground for bring-up:

```bash
sudo /usr/local/sbin/dw3k_daemon -f -c /etc/dw3k-daemon.conf
```

For background mode (systemd/upstart/rc): drop the `-f` flag. A sample systemd
unit can simply `ExecStart=/usr/local/sbin/dw3k_daemon -c /etc/dw3k-daemon.conf`.

Once running, interact via the Unix socket (default `/var/run/dw3k-daemon.sock`).
Each request is a single JSON object terminated by EOF; the daemon replies with
`{"status":"ok",...}` or `{"status":"error","error":"..."}`.

## Commands

| Command (`cmd`)        | JSON Fields                                                                         | Description                                                                                                   |
| ---------------------- | ----------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| `chip_enable`          | –                                                                                   | Initialize hardware, start monitoring.                                                                        |
| `chip_disable`         | –                                                                                   | Stop monitoring and power down DW3K (fails if busy).                                                          |
| `chip_test`            | –                                                                                   | Returns device ID if the chip is active.                                                                      |
| `get_status`           | –                                                                                   | Health info, config echo and last measurement stats.                                                          |
| `get_config`           | –                                                                                   | Raw config values (pan/node IDs, antenna delay, paths).                                                       |
| `set_node_id`          | `node_id`                                                                           | Update local short address.                                                                                   |
| `set_pan_id`           | `pan_id`                                                                            | Update PAN ID.                                                                                                |
| `set_antenna_delay`    | `antenna_delay`                                                                     | Update antenna delay (DWT units).                                                                             |
| `set_monitor_interval` | `interval_ms`                                                                       | Change monitor polling cadence.                                                                               |
| `twr_measure`          | `peer_id`, `role` (`"initiator"`/`"responder"`), `count`, optional `expected_polls` | Run DS-TWR. Initiator role returns measurement stats; responder role blocks until requested polls/stop frame. |

### `twr_measure` Details

- `peer_id` must be a 16-bit ID different from `node_id`.
- When `role` = `initiator` (default):
  - `count` (1–1000) selects how many exchanges to attempt.
  - The response includes raw/filtered statistics plus success counts.
- When `role` = `responder`:
  - `count` is optional; if provided (or `expected_polls`) it limits how many
    polls to serve before exiting.
  - The response simply acknowledges completion.

### Python Helper

```python
import json, socket
SOCK = "/var/run/dw3k-daemon.sock"

def call(payload):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.connect(SOCK)
        s.sendall(json.dumps(payload).encode())
        return json.loads(s.recv(4096))

print(call({"cmd": "chip_enable"}))
print(call({"cmd": "twr_measure", "peer_id": 0x3201,
            "role": "initiator", "count": 50}))
print(call({"cmd": "chip_disable"}))
```

## Testing on SBCs

1. Build and install as above (or cross-compile + copy the binary/config).
2. Edit `/etc/dw3k-daemon.conf` to set the SBC’s `node_id`, `pan_id`,
   `antenna_delay`, and desired socket path.
3. Start the daemon (`-f` for debugging). Verify `get_status` reports
   `chip_active:true` after `chip_enable`.
4. Use the Python helper (or `socat - UNIX-CONNECT:...`) to run
   `twr_measure` against another node. Check the JSON output for range stats.
5. Enable `chip_disable` when idle to extend hardware lifespan.

## License

This project is licensed under the **GNU General Public License v3.0** (GPL v3). See the [LICENSE](LICENSE) file for the full license text.

### Third-Party Components

#### Qorvo DW3000 Driver

The `dwt_uwb_driver/` directory contains code from the Qorvo SDK and is licensed separately:

- **Source**: Qorvo US, Inc. DW3_QM33_SDK_1.1.1
- **License**: LicenseRef-QORVO-2
- **Location**: `dwt_uwb_driver/`
- **Copyright**: Copyright (c) 2024, Qorvo US, Inc.
- **Restrictions**: Can only be used with Qorvo integrated circuits

See [LICENSE-QORVO.txt](LICENSE-QORVO.txt) for full terms.
