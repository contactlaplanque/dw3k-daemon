#!/usr/bin/env python3
"""
Simple CLI client for dw3k-daemon.

Each subcommand assembles a JSON payload, sends it over the Unix domain socket,
then prints the daemon response. All operations rely solely on the Python
standard library so the tool can run on minimal SBC images.
"""

import argparse
import json
import os
import socket
import sys
from typing import Any, Dict

DEFAULT_SOCKET = os.environ.get("DW3K_DAEMON_SOCKET", "/var/run/dw3k-daemon.sock")


def _read_socket_response(sock: socket.socket) -> str:
    chunks = []
    while True:
        data = sock.recv(4096)
        if not data:
            break
        chunks.append(data)
    return b"".join(chunks).decode("utf-8")


def call_daemon(socket_path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
    message = json.dumps(payload).encode("utf-8")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.connect(socket_path)
        sock.sendall(message)
        sock.shutdown(socket.SHUT_WR)
        response = _read_socket_response(sock)
    try:
        return json.loads(response)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Invalid JSON reply: {response}") from exc


def ensure_success(resp: Dict[str, Any]) -> Dict[str, Any]:
    if resp.get("status") != "ok":
        raise SystemExit(f"Daemon error: {resp.get('error', 'unknown error')}")
    return resp.get("data") or {}


def parse_u16(value: str) -> int:
    parsed = int(value, 0)
    if not (0 < parsed <= 0xFFFF):
        raise argparse.ArgumentTypeError("value must be 1-65535 (0x0001-0xFFFF)")
    return parsed


def parse_u32(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def pretty_print(data: Dict[str, Any]) -> None:
    print(json.dumps(data, indent=2, sort_keys=True))


def cmd_simple(args: argparse.Namespace, cmd: str, extra: Dict[str, Any] = None) -> None:
    payload = {"cmd": cmd}
    if extra:
        payload.update(extra)
    resp = call_daemon(args.socket, payload)
    pretty_print(ensure_success(resp))


def cmd_twr_measure(args: argparse.Namespace) -> None:
    payload: Dict[str, Any] = {
        "cmd": "twr_measure",
        "peer_id": args.peer_id,
        "role": args.role,
    }
    if args.count is not None:
        payload["count"] = args.count
    if args.expected_polls is not None:
        payload["expected_polls"] = args.expected_polls
    resp = call_daemon(args.socket, payload)
    pretty_print(ensure_success(resp))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="dw3k-daemon UNIX socket client")
    parser.add_argument(
        "--socket",
        default=DEFAULT_SOCKET,
        help=f"Path to dw3k-daemon socket (default: {DEFAULT_SOCKET})",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("chip-enable", help="Power up chip and start monitoring").set_defaults(
        func=lambda a: cmd_simple(a, "chip_enable")
    )
    subparsers.add_parser("chip-disable", help="Power down chip").set_defaults(
        func=lambda a: cmd_simple(a, "chip_disable")
    )
    subparsers.add_parser("chip-test", help="Read device ID").set_defaults(
        func=lambda a: cmd_simple(a, "chip_test")
    )
    subparsers.add_parser("get-status", help="Retrieve health/status summary").set_defaults(
        func=lambda a: cmd_simple(a, "get_status")
    )
    subparsers.add_parser("get-config", help="Dump current configuration").set_defaults(
        func=lambda a: cmd_simple(a, "get_config")
    )
    set_node = subparsers.add_parser("set-node-id", help="Update local 16-bit node ID")
    set_node.add_argument("node_id", type=parse_u16)
    set_node.set_defaults(func=lambda a: cmd_simple(a, "set_node_id", {"node_id": a.node_id}))

    set_pan = subparsers.add_parser("set-pan-id", help="Update PAN ID")
    set_pan.add_argument("pan_id", type=parse_u16)
    set_pan.set_defaults(func=lambda a: cmd_simple(a, "set_pan_id", {"pan_id": a.pan_id}))

    set_delay = subparsers.add_parser("set-antenna-delay", help="Update antenna delay")
    set_delay.add_argument("antenna_delay", type=parse_u32)
    set_delay.set_defaults(
        func=lambda a: cmd_simple(a, "set_antenna_delay", {"antenna_delay": a.antenna_delay})
    )

    set_monitor = subparsers.add_parser("set-monitor-interval", help="Update monitor interval (ms)")
    set_monitor.add_argument("interval_ms", type=parse_u32)
    set_monitor.set_defaults(
        func=lambda a: cmd_simple(a, "set_monitor_interval", {"interval_ms": a.interval_ms})
    )

    twr = subparsers.add_parser("twr-measure", help="Run a TWR measurement sequence")
    twr.add_argument("peer_id", type=parse_u16, help="Target peer short address")
    twr.add_argument(
        "--role",
        choices=["initiator", "responder"],
        default="initiator",
        help="Role for this node (default: initiator)",
    )
    twr.add_argument(
        "--count",
        type=parse_u32,
        help="Number of measurements (initiator) or polls to serve (responder).",
    )
    twr.add_argument(
        "--expected-polls",
        type=parse_u32,
        help="Responder-only: stop after this many polls regardless of count.",
    )
    twr.set_defaults(func=cmd_twr_measure)

    subparsers.add_parser("shutdown", help="Gracefully stop the daemon").set_defaults(
        func=lambda a: cmd_simple(a, "shutdown")
    )

    return parser


def main(argv: Any = None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        args.func(args)
    except (ConnectionError, FileNotFoundError) as err:
        parser.error(f"Socket error: {err}")
    except BrokenPipeError:
        parser.error("Connection closed unexpectedly by daemon")


if __name__ == "__main__":
    main()

