#!/usr/bin/env bash
# Run as the robot user to fix future user services. Run with sudo as well to
# persist login limits and update already-running desktop terminals in place.
set -euo pipefail

python3 - <<'PY'
import os
import pwd
import re
import resource
import subprocess
from pathlib import Path

is_root = os.geteuid() == 0
username = os.environ.get("SUDO_USER") if is_root else pwd.getpwuid(os.getuid()).pw_name
if not username or username == "root":
    raise SystemExit("Run as the robot user, or use sudo from that user's account.")
account = pwd.getpwnam(username)
uid, gid = account.pw_uid, account.pw_gid

def save(path, content, user_owned=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    previous = path.read_text() if path.exists() else None
    if previous != content:
        if previous is not None:
            backup = path.with_name(path.name + ".before-zhishu-memlock")
            if not backup.exists():
                backup.write_text(previous)
                if user_owned and is_root:
                    os.chown(backup, uid, gid)
        path.write_text(content)
    path.chmod(0o644)
    if user_owned and is_root:
        os.chown(path, uid, gid)
    print("Configured:", path, flush=True)

# systemd 255 reads this user-specific main file. Raising only user@UID.service
# leaves DefaultLimitMEMLOCK finite for newly launched terminal services.
config_dir = Path(account.pw_dir) / ".config" / "systemd"
config = config_dir / "user.conf"
original = config.read_text() if config.exists() else ""
lines = original.splitlines(keepends=True)
result = []
manager = False
found = False
for line in lines:
    section = re.match(r"\s*\[([^]]+)\]", line)
    if section:
        manager = section.group(1) == "Manager"
        result.append(line if line.endswith("\n") else line + "\n")
        if manager and not found:
            result.append("DefaultLimitMEMLOCK=infinity\n")
            found = True
        continue
    if manager and re.match(r"\s*DefaultLimitMEMLOCK\s*=", line):
        continue
    result.append(line)
if not found:
    if result and not result[-1].endswith("\n"):
        result[-1] += "\n"
    result.append("\n[Manager]\nDefaultLimitMEMLOCK=infinity\n")
save(config, "".join(result), user_owned=True)
if is_root:
    # Only directories created for this account need their ownership corrected.
    for directory in (config_dir, config_dir.parent):
        if directory.stat().st_uid == 0:
            os.chown(directory, uid, gid)
    save(Path(f"/etc/security/limits.d/99-zhishu-robot-memlock.conf"),
         f"{username} soft memlock unlimited\n{username} hard memlock unlimited\n")
    save(Path(f"/etc/systemd/system/user@{uid}.service.d/90-zhishu-memlock.conf"),
         "[Service]\nLimitMEMLOCK=infinity\n")
    subprocess.run(["systemctl", "daemon-reload"], check=True)
    # Updating defaults cannot change existing process limits. Repair those too,
    # including gnome-terminal-server and shells that would otherwise reintroduce
    # the finite hard limit. This reserves no memory and restarts no controllers.
    changed = 0
    for directory in Path("/proc").glob("[0-9]*"):
        try:
            status = dict(line.split(":", 1) for line in
                          (directory / "status").read_text().splitlines() if ":" in line)
            if int(status["Uid"].split()[0]) != uid:
                continue
            pid = int(directory.name)
            if resource.prlimit(pid, resource.RLIMIT_MEMLOCK) != (-1, -1):
                resource.prlimit(pid, resource.RLIMIT_MEMLOCK, (-1, -1))
                changed += 1
        except ProcessLookupError:
            pass
        except FileNotFoundError:
            pass
    print("Existing user processes updated:", changed, flush=True)

runtime = Path(f"/run/user/{uid}")
if not runtime.is_dir():
    print("No running user manager; configuration takes effect at next login.")
    raise SystemExit(0)
env = dict(os.environ, XDG_RUNTIME_DIR=str(runtime),
           DBUS_SESSION_BUS_ADDRESS=f"unix:path={runtime}/bus")
prefix = ["runuser", "-u", username, "--"] if is_root else []
# Reload reads manager defaults without re-executing the old manager, which
# would restore the finite hard limit saved when this desktop session started.
subprocess.run(prefix + ["systemctl", "--user", "daemon-reload"], env=env, check=True)
subprocess.run(prefix + ["systemctl", "--user", "show",
                        "-p", "DefaultLimitMEMLOCK", "-p", "DefaultLimitMEMLOCKSoft"],
               env=env, check=True)
# Verify the default inherited by a fresh service, without overriding its limit.
probe = "import resource; x=resource.getrlimit(resource.RLIMIT_MEMLOCK); print('New service memlock:', x); raise SystemExit(0 if x == (-1,-1) else 1)"
verification = subprocess.run(prefix + ["systemd-run", "--user", "--quiet", "--wait", "--pipe", "--collect",
                                       "/usr/bin/python3", "-c", probe], env=env)
if verification.returncode:
    raise SystemExit("Configuration saved. Run this script once with sudo to update the current user manager and old terminals; new-service verification has not passed yet.")
print("Persistent memlock configuration verified. No robot motion commands sent.")
if not is_root:
    print("Existing finite-limit terminals keep their old limits; sudo running this script updates them in place.")
PY
