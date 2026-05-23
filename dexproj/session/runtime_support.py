"""Runtime support for DexProj session orchestration."""

from __future__ import annotations

import json
import os
import select
import shlex
import signal
import subprocess
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from pathlib import Path


@dataclass
class ManagedProcessSpec:
    name: str
    command: list[str]
    cwd: Path
    env: dict[str, str] | None = None
    required: bool = True
    stdout_path: Path | None = None
    stderr_path: Path | None = None


@dataclass
class ManagedProcessHandle:
    spec: ManagedProcessSpec
    process: subprocess.Popen[str]
    stdout_handle: object | None = None
    stderr_handle: object | None = None


class ManagedProcessGroup:
    def __init__(self, specs: list[ManagedProcessSpec] | None = None, dry_run: bool = False):
        self.specs = specs or []
        self.dry_run = dry_run
        self.handles: list[ManagedProcessHandle] = []

    def start_all(self) -> None:
        for spec in self.specs:
            self._start_one(spec)

    def start_specs(self, specs: list[ManagedProcessSpec]) -> None:
        for spec in specs:
            self._start_one(spec)

    def _start_one(self, spec: ManagedProcessSpec) -> None:
        pretty = " ".join(shlex.quote(token) for token in spec.command)
        print(f"[proc] start {spec.name}: {pretty}")
        if self.dry_run:
            return
        env = os.environ.copy()
        if spec.env:
            env.update(spec.env)
        stdout_handle = self._open_log_handle(spec.stdout_path)
        stderr_handle = self._open_log_handle(spec.stderr_path)
        process = subprocess.Popen(
            spec.command,
            cwd=str(spec.cwd),
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=stdout_handle if stdout_handle is not None else None,
            stderr=stderr_handle if stderr_handle is not None else None,
            text=True,
            start_new_session=True,
        )
        self.handles.append(
            ManagedProcessHandle(
                spec=spec,
                process=process,
                stdout_handle=stdout_handle,
                stderr_handle=stderr_handle,
            )
        )

    def poll_failures(self) -> list[str]:
        failures: list[str] = []
        for handle in self.handles:
            code = handle.process.poll()
            if code is None:
                continue
            if code != 0:
                failures.append(f"{handle.spec.name} exited with code {code}")
        return failures

    def snapshot(self) -> list[dict]:
        snapshots: list[dict] = []
        for handle in self.handles:
            code = handle.process.poll()
            snapshots.append(
                {
                    "name": handle.spec.name,
                    "pid": handle.process.pid,
                    "alive": code is None,
                    "returncode": code,
                    "command": list(handle.spec.command),
                    "cwd": str(handle.spec.cwd),
                }
            )
        return snapshots

    def stop_all(self, grace_sec: float = 5.0) -> None:
        if self.dry_run:
            return
        alive: list[ManagedProcessHandle] = []
        for handle in reversed(self.handles):
            if handle.process.poll() is None:
                alive.append(handle)
                self._signal_group(handle.process.pid, signal.SIGINT)
        deadline = time.time() + grace_sec
        while alive and time.time() < deadline:
            alive = [handle for handle in alive if handle.process.poll() is None]
            if alive:
                time.sleep(0.1)
        for handle in alive:
            self._signal_group(handle.process.pid, signal.SIGTERM)
        deadline = time.time() + max(grace_sec / 2.0, 1.0)
        while alive and time.time() < deadline:
            alive = [handle for handle in alive if handle.process.poll() is None]
            if alive:
                time.sleep(0.1)
        for handle in alive:
            self._signal_group(handle.process.pid, signal.SIGKILL)
        for handle in reversed(self.handles):
            try:
                handle.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
            self._close_handle(handle.stdout_handle)
            self._close_handle(handle.stderr_handle)

    @staticmethod
    def _signal_group(pid: int, sig: signal.Signals) -> None:
        try:
            os.killpg(pid, sig)
        except ProcessLookupError:
            return

    @staticmethod
    def _open_log_handle(path: Path | None):
        if path is None:
            return None
        path.parent.mkdir(parents=True, exist_ok=True)
        return path.open("a", encoding="utf-8")

    @staticmethod
    def _close_handle(handle: object | None) -> None:
        if handle is None:
            return
        close = getattr(handle, "close", None)
        if callable(close):
            close()


class PeriodicStatusWriter:
    def __init__(self, output_path: Path, interval_sec: float = 0.5):
        self.output_path = output_path
        self.interval_sec = max(float(interval_sec), 0.1)
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    def start(self, snapshot_fn) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(
            target=self._run,
            args=(snapshot_fn,),
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self, snapshot_fn) -> None:
        while not self._stop_event.is_set():
            payload = snapshot_fn()
            payload.setdefault("timestamp_unix", time.time())
            with self.output_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(payload, ensure_ascii=False) + "\n")
            self._stop_event.wait(self.interval_sec)


class TriggerError(RuntimeError):
    pass


class BaseTrigger:
    def wait_for_start(self) -> str:
        raise NotImplementedError

    def wait_for_stop(self) -> str:
        raise NotImplementedError


class KeyboardTrigger(BaseTrigger):
    def __init__(self, start_key: str, stop_key: str):
        self.start_key = start_key.strip().upper()
        self.stop_key = stop_key.strip().upper()

    def wait_for_start(self) -> str:
        print(f"[trigger] keyboard start key: {self.start_key}")
        return self._wait_for_key(self.start_key, prompt="start")

    def wait_for_stop(self) -> str:
        print(f"[trigger] keyboard stop key: {self.stop_key}")
        return self._wait_for_key(self.stop_key, prompt="stop")

    def _wait_for_key(self, expected: str, prompt: str) -> str:
        if not sys.stdin.isatty():
            raise TriggerError("Keyboard trigger requires an interactive TTY.")
        print(f"Press {expected} to {prompt}...", flush=True)
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                ready, _, _ = select.select([fd], [], [], 0.2)
                if not ready:
                    continue
                value = os.read(fd, 1).decode("utf-8", errors="ignore")
                if not value:
                    continue
                key = value.upper()
                if key == expected:
                    print(f"[trigger] keyboard -> {key}")
                    return key
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


class InputsGamepadTrigger(BaseTrigger):
    def __init__(self, start_buttons: list[str], stop_buttons: list[str]):
        try:
            from inputs import get_gamepad  # type: ignore
        except ImportError as exc:  # pragma: no cover
            raise TriggerError(
                "Gamepad trigger requires the Python package `inputs`. "
                "Install it in the active environment to use trigger_mode=gamepad or both."
            ) from exc
        self._get_gamepad = get_gamepad
        self.start_buttons = {self._normalize_button(name) for name in start_buttons}
        self.stop_buttons = {self._normalize_button(name) for name in stop_buttons}

    def wait_for_start(self) -> str:
        print(f"[trigger] gamepad start buttons: {sorted(self.start_buttons)}")
        return self._wait_for_buttons(self.start_buttons, phase="start")

    def wait_for_stop(self) -> str:
        print(f"[trigger] gamepad stop buttons: {sorted(self.stop_buttons)}")
        return self._wait_for_buttons(self.stop_buttons, phase="stop")

    def _wait_for_buttons(self, expected: set[str], phase: str) -> str:
        print(f"Press gamepad {', '.join(sorted(expected))} to {phase}...", flush=True)
        while True:
            events = self._get_gamepad()
            for event in events:
                if event.ev_type != "Key":
                    continue
                if int(event.state) != 1:
                    continue
                button = self._normalize_code(event.code)
                if button in expected:
                    print(f"[trigger] gamepad -> {button}")
                    return button

    @staticmethod
    def _normalize_button(name: str) -> str:
        return name.strip().lower()

    @staticmethod
    def _normalize_code(code: str) -> str:
        mapped = {
            "BTN_TL": "lb",
            "BTN_TR": "rb",
            "BTN_START": "start",
            "BTN_SELECT": "back",
            "BTN_MODE": "home",
            "BTN_SOUTH": "a",
            "BTN_EAST": "b",
            "BTN_NORTH": "x",
            "BTN_WEST": "y",
            "BTN_THUMBL": "lo",
            "BTN_THUMBR": "ro",
        }
        return mapped.get(code, code.strip().lower())


class CombinedTrigger(BaseTrigger):
    def __init__(self, primary: BaseTrigger, secondary: BaseTrigger):
        self.primary = primary
        self.secondary = secondary

    def wait_for_start(self) -> str:
        return _race_triggers(self.primary.wait_for_start, self.secondary.wait_for_start)

    def wait_for_stop(self) -> str:
        return _race_triggers(self.primary.wait_for_stop, self.secondary.wait_for_stop)


def _race_triggers(primary_wait, secondary_wait) -> str:
    result: dict[str, str] = {}
    errors: list[BaseException] = []
    done = threading.Event()

    def runner(name: str, fn) -> None:
        try:
            value = fn()
            if not done.is_set():
                result[name] = value
                done.set()
        except BaseException as exc:  # pragma: no cover
            errors.append(exc)
            if len(errors) >= 2:
                done.set()

    threads = [
        threading.Thread(target=runner, args=("primary", primary_wait), daemon=True),
        threading.Thread(target=runner, args=("secondary", secondary_wait), daemon=True),
    ]
    for thread in threads:
        thread.start()
    while not done.wait(0.1):
        continue
    if result:
        return next(iter(result.values()))
    raise TriggerError("All trigger backends failed: " + "; ".join(str(err) for err in errors))
