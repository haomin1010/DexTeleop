"""Sync recorded episodes from the DexProj container back to the host workspace."""

from __future__ import annotations

import os
from pathlib import Path

SYNC_MARKER_REL = Path("data/.sync_episode_request")


def request_episode_sync(episode_dir: Path) -> None:
    """Write a marker file so the host-side watcher can docker-cp this episode."""
    if not _running_in_container():
        return
    data_root = Path("data").resolve()
    episode_path = episode_dir.resolve()
    try:
        rel_episode = episode_path.relative_to(data_root)
    except ValueError:
        print(f"[session] skip host sync: episode not under {data_root}: {episode_path}")
        return
    marker_path = SYNC_MARKER_REL
    marker_path.parent.mkdir(parents=True, exist_ok=True)
    marker_path.write_text(f"{rel_episode.as_posix()}\n", encoding="utf-8")
    print(f"[session] queued host sync: data/{rel_episode.as_posix()}")


def _running_in_container() -> bool:
    return os.environ.get("DEXPROJ_RUNNING_IN_CONTAINER") == "1" or Path("/.dockerenv").is_file()
