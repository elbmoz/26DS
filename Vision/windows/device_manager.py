"""SSH/SFTP lifecycle manager for the MaixCAM vision application.

The manager deliberately owns only processes started from ``MANAGED_ROOT``.
It never kills a process discovered by name, so MaixVision and unrelated
device applications cannot be terminated accidentally.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import posixpath
import socket
import time
import uuid

import paramiko


MANAGED_ROOT = "/root/pipe_ball_vision"
RELEASES_DIR = posixpath.join(MANAGED_ROOT, "releases")
RUNTIME_DIR = posixpath.join(MANAGED_ROOT, "runtime")
CURRENT_LINK = posixpath.join(MANAGED_ROOT, "current")
PID_FILE = posixpath.join(RUNTIME_DIR, "vision.pid")
LOG_FILE = posixpath.join(RUNTIME_DIR, "device.log")
MANIFEST_NAME = "deploy_manifest.json"

SOURCE_SUFFIXES = {".py", ".jpg", ".jpeg", ".png", ".json"}


class DeviceManagerError(RuntimeError):
    pass


def source_files(source_dir):
    source_dir = Path(source_dir).resolve()
    files = []
    for path in source_dir.rglob("*"):
        if not path.is_file():
            continue
        relative = path.relative_to(source_dir)
        if "__pycache__" in relative.parts or path.suffix == ".pyc":
            continue
        if path.suffix.lower() not in SOURCE_SUFFIXES:
            continue
        files.append(path)
    return sorted(files, key=lambda item: item.relative_to(source_dir).as_posix())


def build_manifest(source_dir):
    source_dir = Path(source_dir).resolve()
    files = source_files(source_dir)
    if not files:
        raise DeviceManagerError("vision source directory is empty")
    if not (source_dir / "main.py").is_file():
        raise DeviceManagerError("main.py is missing from vision source")

    entries = {}
    release_hash = hashlib.sha256()
    for path in files:
        relative = path.relative_to(source_dir).as_posix()
        payload = path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        entries[relative] = {
            "sha256": digest,
            "bytes": len(payload),
        }
        release_hash.update(relative.encode("utf-8"))
        release_hash.update(b"\0")
        release_hash.update(bytes.fromhex(digest))

    release_id = release_hash.hexdigest()[:16]
    return {
        "schema": 1,
        "release_id": release_id,
        "source_hash": release_hash.hexdigest(),
        "files": entries,
    }


def preflight_source(source_dir):
    source_dir = Path(source_dir).resolve()
    manifest = build_manifest(source_dir)
    checked = 0
    for path in source_files(source_dir):
        if path.suffix != ".py":
            continue
        compile(
            path.read_text(encoding="utf-8"),
            str(path),
            "exec",
            dont_inherit=True,
        )
        checked += 1
    return {
        "ok": True,
        "python_files_checked": checked,
        "release_id": manifest["release_id"],
        "manifest": manifest,
    }


class MaixCamDeviceManager:
    def __init__(
        self,
        host="10.16.6.1",
        username="root",
        password=None,
        port=22,
        timeout=6.0,
    ):
        self.host = str(host)
        self.username = str(username)
        self.password = (
            os.environ.get("MAIXCAM_PASSWORD", "root")
            if password is None
            else str(password)
        )
        self.port = int(port)
        self.timeout = float(timeout)
        self._client = None
        self._sftp = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()

    def connect(self):
        if self._client is not None:
            return
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(
                self.host,
                port=self.port,
                username=self.username,
                password=self.password,
                timeout=self.timeout,
                banner_timeout=self.timeout,
                auth_timeout=self.timeout,
                look_for_keys=False,
                allow_agent=False,
            )
            self._sftp = client.open_sftp()
        except Exception as exc:
            client.close()
            raise DeviceManagerError(
                "cannot connect to MaixCAM {}:{}: {}".format(
                    self.host, self.port, exc
                )
            ) from exc
        self._client = client

    def close(self):
        if self._sftp is not None:
            self._sftp.close()
        if self._client is not None:
            self._client.close()
        self._sftp = None
        self._client = None

    def _ensure_connected(self):
        if self._client is None:
            self.connect()

    def _exec(self, command, timeout=None, check=True):
        self._ensure_connected()
        _, stdout, stderr = self._client.exec_command(
            command,
            timeout=self.timeout if timeout is None else float(timeout),
        )
        output = stdout.read().decode("utf-8", errors="replace")
        error = stderr.read().decode("utf-8", errors="replace")
        exit_code = stdout.channel.recv_exit_status()
        if check and exit_code != 0:
            raise DeviceManagerError(
                "remote command failed ({}): {}\n{}".format(
                    exit_code, command, error.strip()
                )
            )
        return exit_code, output, error

    def _mkdirs(self, remote_path):
        self._ensure_connected()
        current = ""
        for part in remote_path.strip("/").split("/"):
            current += "/" + part
            try:
                self._sftp.stat(current)
            except OSError:
                self._sftp.mkdir(current, mode=0o755)

    def _read_remote_json(self, path):
        self._ensure_connected()
        try:
            with self._sftp.open(path, "r") as handle:
                return json.loads(handle.read().decode("utf-8"))
        except (OSError, ValueError, json.JSONDecodeError):
            return None

    def _write_remote_json(self, path, value):
        temporary = path + ".tmp-" + uuid.uuid4().hex[:8]
        payload = json.dumps(
            value, ensure_ascii=False, indent=2, sort_keys=True
        ).encode("utf-8")
        with self._sftp.open(temporary, "wb") as handle:
            handle.write(payload)
        self._sftp.chmod(temporary, 0o644)
        self._sftp.rename(temporary, path)

    def _activate_release(self, release_id):
        relative_target = "releases/" + str(release_id)
        temporary = CURRENT_LINK + ".next-" + uuid.uuid4().hex[:8]
        self._sftp.symlink(relative_target, temporary)
        try:
            self._sftp.posix_rename(temporary, CURRENT_LINK)
        except (AttributeError, OSError):
            try:
                self._sftp.remove(CURRENT_LINK)
            except OSError:
                pass
            self._sftp.rename(temporary, CURRENT_LINK)

    def deploy(self, source_dir):
        source_dir = Path(source_dir).resolve()
        checked = preflight_source(source_dir)
        manifest = checked["manifest"]
        release_id = manifest["release_id"]
        release_dir = posixpath.join(RELEASES_DIR, release_id)
        remote_manifest = posixpath.join(release_dir, MANIFEST_NAME)

        self._mkdirs(RELEASES_DIR)
        self._mkdirs(RUNTIME_DIR)
        existing = self._read_remote_json(remote_manifest)
        uploaded = False
        if (
            not isinstance(existing, dict)
            or existing.get("source_hash") != manifest["source_hash"]
        ):
            staging = posixpath.join(
                MANAGED_ROOT,
                ".staging-{}-{}".format(release_id, uuid.uuid4().hex[:8]),
            )
            self._mkdirs(staging)
            for local_path in source_files(source_dir):
                relative = local_path.relative_to(source_dir).as_posix()
                remote_path = posixpath.join(staging, relative)
                self._mkdirs(posixpath.dirname(remote_path))
                self._sftp.put(str(local_path), remote_path)
                self._sftp.chmod(remote_path, 0o644)
            deployed_manifest = dict(manifest)
            deployed_manifest["deployed_epoch_ns"] = time.time_ns()
            self._write_remote_json(
                posixpath.join(staging, MANIFEST_NAME),
                deployed_manifest,
            )
            try:
                self._sftp.rename(staging, release_dir)
            except OSError as exc:
                existing = self._read_remote_json(remote_manifest)
                if (
                    not isinstance(existing, dict)
                    or existing.get("source_hash")
                    != manifest["source_hash"]
                ):
                    raise DeviceManagerError(
                        "could not publish remote release: {}".format(exc)
                    ) from exc
            uploaded = True

        self._activate_release(release_id)
        return {
            "ok": True,
            "host": self.host,
            "release_id": release_id,
            "source_hash": manifest["source_hash"],
            "uploaded": uploaded,
            "python_files_checked": checked["python_files_checked"],
            "remote_directory": release_dir,
        }

    def _read_pid(self):
        self._ensure_connected()
        try:
            with self._sftp.open(PID_FILE, "r") as handle:
                value = handle.read().decode("ascii", errors="ignore").strip()
        except OSError:
            return None
        return int(value) if value.isdigit() else None

    def _process_info(self, pid):
        if pid is None:
            return None
        process_root = "/proc/{}".format(int(pid))
        try:
            self._sftp.stat(process_root)
            with self._sftp.open(
                posixpath.join(process_root, "cmdline"), "rb"
            ) as handle:
                command = (
                    handle.read()
                    .replace(b"\0", b" ")
                    .decode("utf-8", errors="replace")
                    .strip()
                )
            _, cwd, _ = self._exec(
                "readlink /proc/{}/cwd".format(int(pid)),
                check=False,
            )
        except OSError:
            return None
        cwd = cwd.strip()
        managed = (
            cwd.startswith(RELEASES_DIR + "/")
            and "python3" in command
            and "main.py" in command
        )
        return {
            "pid": int(pid),
            "command": command,
            "cwd": cwd,
            "managed": managed,
        }

    def current_release(self):
        _, output, _ = self._exec(
            "readlink -f {}".format(CURRENT_LINK),
            check=False,
        )
        resolved = output.strip()
        if not resolved.startswith(RELEASES_DIR + "/"):
            return None
        return posixpath.basename(resolved)

    def log_tail(self, lines=40):
        lines = max(1, min(1000, int(lines)))
        _, output, _ = self._exec(
            "tail -n {} {}".format(lines, LOG_FILE),
            check=False,
        )
        return output

    def status(self, log_lines=12):
        pid = self._read_pid()
        process = self._process_info(pid)
        release_id = self.current_release()
        manifest = (
            self._read_remote_json(
                posixpath.join(RELEASES_DIR, release_id, MANIFEST_NAME)
            )
            if release_id
            else None
        )
        return {
            "ok": True,
            "host": self.host,
            "ssh": True,
            "running": bool(process and process["managed"]),
            "process": process,
            "current_release": release_id,
            "source_hash": (
                manifest.get("source_hash")
                if isinstance(manifest, dict)
                else None
            ),
            "log_file": LOG_FILE,
            "log_tail": self.log_tail(log_lines),
        }

    def start(self, wait_seconds=4.0):
        status = self.status(log_lines=4)
        if status["running"]:
            return {
                "ok": True,
                "already_running": True,
                **status,
            }
        if status["process"] and not status["process"]["managed"]:
            raise DeviceManagerError(
                "PID file points to an unmanaged process; refusing to start"
            )
        release_id = status["current_release"]
        if not release_id:
            raise DeviceManagerError("no deployed release is active")

        self._mkdirs(RUNTIME_DIR)
        command = (
            "cd {current} && "
            # MaixCAM's launcher daemon remains active for SSH-started apps
            # and can consume about a quarter of the single CPU. Give the
            # control-critical vision loop scheduling priority while leaving
            # the launcher alive and responsive.
            "nohup nice -n -5 python3 -u main.py "
            "</dev/null >{log} 2>&1 & "
            "pid=$!; echo $pid >{pid_tmp}; mv {pid_tmp} {pid_file}; echo $pid"
        ).format(
            current=CURRENT_LINK,
            log=LOG_FILE,
            pid_tmp=PID_FILE + ".tmp",
            pid_file=PID_FILE,
        )
        _, output, _ = self._exec(command, timeout=10)
        started_pid = int(output.strip().splitlines()[-1])
        deadline = time.monotonic() + max(0.5, float(wait_seconds))
        process = None
        while time.monotonic() < deadline:
            process = self._process_info(started_pid)
            if process and process["managed"]:
                time.sleep(0.25)
            else:
                break
            if "RTSP stream:" in self.log_tail(30):
                break
        process = self._process_info(started_pid)
        if not process or not process["managed"]:
            raise DeviceManagerError(
                "vision process exited during startup:\n{}".format(
                    self.log_tail(80)
                )
            )
        return {
            "ok": True,
            "already_running": False,
            "pid": started_pid,
            "release_id": release_id,
            "log_tail": self.log_tail(30),
        }

    def stop(self, timeout=6.0):
        pid = self._read_pid()
        process = self._process_info(pid)
        if process is None:
            try:
                self._sftp.remove(PID_FILE)
            except OSError:
                pass
            return {"ok": True, "already_stopped": True}
        if not process["managed"]:
            raise DeviceManagerError(
                "refusing to stop unmanaged PID {} ({})".format(
                    pid, process["command"]
                )
            )

        self._exec("kill -INT {}".format(pid), check=False)
        deadline = time.monotonic() + max(0.5, float(timeout))
        while time.monotonic() < deadline:
            if self._process_info(pid) is None:
                break
            time.sleep(0.20)
        forced = False
        if self._process_info(pid) is not None:
            forced = True
            self._exec("kill -KILL {}".format(pid), check=False)
            time.sleep(0.30)
        try:
            self._sftp.remove(PID_FILE)
        except OSError:
            pass
        return {
            "ok": True,
            "already_stopped": False,
            "pid": pid,
            "forced": forced,
            "log_tail": self.log_tail(20),
        }

    def restart(self, wait_seconds=4.0):
        stopped = self.stop()
        started = self.start(wait_seconds=wait_seconds)
        return {"ok": True, "stop": stopped, "start": started}

    def releases(self):
        self._ensure_connected()
        try:
            names = self._sftp.listdir(RELEASES_DIR)
        except OSError:
            names = []
        current = self.current_release()
        results = []
        for name in sorted(names, reverse=True):
            manifest = self._read_remote_json(
                posixpath.join(RELEASES_DIR, name, MANIFEST_NAME)
            )
            if not isinstance(manifest, dict):
                continue
            results.append(
                {
                    "release_id": name,
                    "current": name == current,
                    "source_hash": manifest.get("source_hash"),
                    "deployed_epoch_ns": manifest.get(
                        "deployed_epoch_ns"
                    ),
                }
            )
        return results

    def rollback(self, release_id):
        release_id = str(release_id)
        if (
            not release_id
            or "/" in release_id
            or "\\" in release_id
            or release_id in (".", "..")
        ):
            raise DeviceManagerError("invalid release id")
        manifest = self._read_remote_json(
            posixpath.join(RELEASES_DIR, release_id, MANIFEST_NAME)
        )
        if not isinstance(manifest, dict):
            raise DeviceManagerError(
                "release does not exist: {}".format(release_id)
            )
        if self.status(log_lines=0)["running"]:
            raise DeviceManagerError(
                "stop the managed process before changing releases"
            )
        self._activate_release(release_id)
        return {
            "ok": True,
            "release_id": release_id,
            "source_hash": manifest.get("source_hash"),
        }


def tcp_reachable(host, port, timeout=1.0):
    try:
        with socket.create_connection(
            (str(host), int(port)), timeout=float(timeout)
        ):
            return True
    except OSError:
        return False
