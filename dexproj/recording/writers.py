"""Pluggable episode writers for DexProj."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Protocol


class EpisodeWriter(Protocol):
    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...


@dataclass
class JsonlWriter:
    path: Path
    snapshot_fn: Callable[[], dict]
    interval_sec: float = 0.5
    _thread: threading.Thread | None = field(default=None, init=False)
    _stop_event: threading.Event = field(default_factory=threading.Event, init=False)

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            payload = self.snapshot_fn()
            payload.setdefault("timestamp_unix", time.time())
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(payload, ensure_ascii=False) + "\n")
            self._stop_event.wait(self.interval_sec)


@dataclass
class PlaceholderFileWriter:
    path: Path
    title: str
    body: str

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(self.body + "\n", encoding="utf-8")

    def stop(self) -> None:
        return
