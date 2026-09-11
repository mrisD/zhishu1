#!/usr/bin/env python3

import argparse
import signal
import sys
import time

import rclpy
from std_srvs.srv import SetBool


def call_both(node, clients, enabled, timeout):
    request = SetBool.Request()
    request.data = enabled
    futures = [client.call_async(request) for client in clients]
    deadline = time.monotonic() + timeout
    while not all(future.done() for future in futures):
        if time.monotonic() >= deadline:
            raise TimeoutError("timed out waiting for PDO recording services")
        rclpy.spin_once(node, timeout_sec=0.05)

    responses = []
    for future in futures:
        response = future.result()
        if response is None:
            raise RuntimeError("PDO recording service returned no response")
        responses.append(response)
    return responses


def main():
    parser = argparse.ArgumentParser(
        description="Record exact 1 kHz PDO values for both robot arms."
    )
    parser.add_argument(
        "duration",
        nargs="?",
        type=float,
        default=20.0,
        help="recording duration in seconds (default: 20)",
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=10.0,
        help="service discovery/call timeout in seconds (default: 10)",
    )
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("duration must be positive")

    rclpy.init()
    node = rclpy.create_node("pdo_diagnostics_recorder")
    service_names = ("/left_arm/record_pdo", "/right_arm/record_pdo")
    clients = [node.create_client(SetBool, name) for name in service_names]
    started = False
    interrupted = False

    def request_stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        deadline = time.monotonic() + args.service_timeout
        for name, client in zip(service_names, clients):
            remaining = max(0.0, deadline - time.monotonic())
            if not client.wait_for_service(timeout_sec=remaining):
                raise TimeoutError(f"service is unavailable: {name}")

        start_responses = call_both(node, clients, True, args.service_timeout)
        for name, response in zip(service_names, start_responses):
            print(f"{name}: {response.message}")
        if not all(response.success for response in start_responses):
            raise RuntimeError("one or more PDO recorders failed to start")
        started = True

        stop_at = time.monotonic() + args.duration
        print(f"Recording both arms for {args.duration:.3f} seconds...")
        while not interrupted and time.monotonic() < stop_at:
            rclpy.spin_once(node, timeout_sec=min(0.1, stop_at - time.monotonic()))
    finally:
        if started:
            try:
                stop_responses = call_both(
                    node, clients, False, args.service_timeout
                )
                for name, response in zip(service_names, stop_responses):
                    print(f"{name}: {response.message}")
            except Exception as error:
                print(f"failed to stop PDO recording cleanly: {error}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()

    return 130 if interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())
