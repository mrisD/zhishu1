#!/usr/bin/env python3
"""Follow pallet workflow stages with an independently owned Gazebo GUI."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


STAGES = ("pilot", "full_nine", "recording")
NETWORK_KEYS = (
    "ROS_DOMAIN_ID", "GZ_PARTITION", "GZ_IP", "GZ_RELAY",
    "ROS_AUTOMATIC_DISCOVERY_RANGE", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI",
)


def parent_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        pass
    try:
        # kill(pid, 0) also succeeds for a parent awaiting reaping.
        status = Path(f"/proc/{pid}/stat").read_text()
        return status.rsplit(")", 1)[1].split()[0] != "Z"
    except (FileNotFoundError, ProcessLookupError):
        return False
    except (PermissionError, IndexError):
        return True


def network_ready(run_dir):
    try:
        config = json.loads((run_dir / "network_config.json").read_text())
        return isinstance(config, dict) and all(
            isinstance(config.get(key), str) for key in NETWORK_KEYS
        )
    except (OSError, ValueError):
        # The runner may still be writing its network configuration.
        return False


def result_ready(run_dir):
    try:
        return isinstance(json.loads((run_dir / "sequence_result.json").read_text()), dict)
    except (OSError, ValueError):
        return False


def close_owned_gui(process):
    """Stop only the process group created for our preceding GUI."""
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3)
    except ProcessLookupError:
        pass
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workflow-dir", type=Path, required=True)
    parser.add_argument("--parent-pid", type=int, required=True)
    args = parser.parse_args()
    if args.parent_pid <= 1:
        parser.error("--parent-pid must be greater than 1")
    workflow = args.workflow_dir.resolve()
    root = Path(__file__).resolve().parent.parent
    deadline = time.monotonic() + 12 * 60 * 60
    current_index = -1
    gui = None
    gui_exit_reported = False
    gui_retries = 0
    retry_after = 0.0
    print(f"WORKFLOW_GUI_MONITOR workflow={workflow} parent_pid={args.parent_pid}", flush=True)
    try:
        while time.monotonic() < deadline:
            if not parent_alive(args.parent_pid):
                print("WORKFLOW_GUI_MONITOR_EXIT reason=parent_exited final_gui_preserved=1", flush=True)
                return
            ready = [i for i, stage in enumerate(STAGES)
                     if i > current_index and network_ready(workflow / stage)]
            if ready:
                next_index = max(ready)
                run_dir = workflow / STAGES[next_index]
                close_owned_gui(gui)
                gui = subprocess.Popen(
                    ["bash", str(root / "scripts/run_with_gz_control_safe_overlay.sh"),
                     "python3", str(root / "scripts/open_pallet_sim_gui.py"),
                     "--run-dir", str(run_dir)],
                    cwd=root, start_new_session=True,
                )
                current_index = next_index
                gui_exit_reported = False
                gui_retries = 0
                print(f"WORKFLOW_GUI_SWITCH stage={STAGES[current_index]} gui_pid={gui.pid} run_dir={run_dir}", flush=True)
            if gui is not None and gui.poll() is not None and not gui_exit_reported:
                print(f"WORKFLOW_GUI_EXIT stage={STAGES[current_index]} returncode={gui.returncode}", flush=True)
                gui_exit_reported = True
                retry_after = time.monotonic() + 3
            if (gui is not None and gui.poll() not in (None, 0) and gui_exit_reported and
                    gui_retries < 3 and time.monotonic() >= retry_after):
                gui_retries += 1
                gui = subprocess.Popen(
                    ["bash", str(root / "scripts/run_with_gz_control_safe_overlay.sh"),
                     "python3", str(root / "scripts/open_pallet_sim_gui.py"),
                     "--run-dir", str(workflow / STAGES[current_index])],
                    cwd=root, start_new_session=True,
                )
                gui_exit_reported = False
                print(f"WORKFLOW_GUI_RETRY stage={STAGES[current_index]} attempt={gui_retries} gui_pid={gui.pid}", flush=True)
            if result_ready(workflow / "recording"):
                print("WORKFLOW_GUI_MONITOR_EXIT reason=recording_finished final_gui_preserved=1", flush=True)
                return
            time.sleep(2)
        print("WORKFLOW_GUI_MONITOR_EXIT reason=12_hour_limit final_gui_preserved=1", flush=True)
    except KeyboardInterrupt:
        print("WORKFLOW_GUI_MONITOR_EXIT reason=interrupted final_gui_preserved=1", flush=True)


if __name__ == "__main__":
    main()
