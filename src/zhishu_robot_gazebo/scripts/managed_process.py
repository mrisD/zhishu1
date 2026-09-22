#!/usr/bin/env python3
"""Run a child process and make known-broken simulation shutdowns deterministic.

MoveIt 2.12.x on ROS 2 Jazzy and the Gazebo Harmonic stack used by this
workspace can crash in their teardown paths.  During a launch shutdown,
terminate the isolated child process group without entering those paths and
let launch observe this supervisor exiting normally.  A child that fails
while the system is running still propagates its real exit status.
"""

import os
import signal
import subprocess
import sys
import time


def main() -> int:
    if len(sys.argv) < 3 or sys.argv[1] != "--managed-target":
        print(
            "usage: managed_process.py --managed-target EXECUTABLE [ARG ...]",
            file=sys.stderr,
        )
        return 2

    command = [sys.argv[2], *sys.argv[3:]]
    child = None
    shutdown_signal = None

    def request_shutdown(signum, _frame):
        nonlocal shutdown_signal
        shutdown_signal = signum
        if child is not None and child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    child = subprocess.Popen(command, start_new_session=True)
    if shutdown_signal is not None and child.poll() is None:
        request_shutdown(shutdown_signal, None)

    while True:
        try:
            return_code = child.wait()
            break
        except InterruptedError:
            continue

    if shutdown_signal is not None:
        print(
            f"[managed_process] {os.path.basename(command[0])} stopped "
            "without entering the known shutdown crash path",
            flush=True,
        )
        return 0

    # Preserve failures that happen during normal operation.
    if return_code < 0:
        return 128 - return_code
    return return_code


if __name__ == "__main__":
    sys.exit(main())
