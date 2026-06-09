"""Isolate libKine.so FK/IK in a child process (spawn).

Parent keeps Marvin SDK only. All FX_Robot_Kine_* calls run in the worker so two
processes never touch libKine.so at the same time.
"""

from __future__ import annotations

import multiprocessing
import os
import queue
import time
from types import SimpleNamespace
from typing import Any, Optional


def _ik_worker_loop(
    config_path: str,
    request_queue: multiprocessing.Queue,
    response_queue: multiprocessing.Queue,
) -> None:
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    old_stdout = os.dup(1)
    old_stderr = os.dup(2)
    os.dup2(devnull_fd, 1)
    os.dup2(devnull_fd, 2)

    try:
        from tianji_output._internal.fx_kine import Marvin_Kine

        kine_left = Marvin_Kine()
        config_result = kine_left.load_config(config_path=config_path)
        time.sleep(0.1)
        kine_left.initial_kine(
            robot_serial=0,
            robot_type=config_result["TYPE"][0],
            dh=config_result["DH"][0],
            pnva=config_result["PNVA"][0],
            j67=config_result["BD"][0],
        )

        kine_right = Marvin_Kine()
        config_result2 = kine_right.load_config(config_path=config_path)
        time.sleep(0.1)
        kine_right.initial_kine(
            robot_serial=1,
            robot_type=config_result2["TYPE"][1],
            dh=config_result2["DH"][1],
            pnva=config_result2["PNVA"][1],
            j67=config_result2["BD"][1],
        )
        kines = {0: kine_left, 1: kine_right}

        response_queue.put({"type": "ready", "pnva": config_result2["PNVA"]})

        while True:
            req = request_queue.get()
            if req is None:
                break
            req_id = req.get("id", 0)
            op = req.get("op", "ik")
            try:
                serial = int(req["robot_serial"])
                kine = kines[serial]
                if op == "fk":
                    fk_mat = kine.fk(serial, list(req["joints"]))
                    if fk_mat is False:
                        response_queue.put({"id": req_id, "ok": True, "sdk_ok": False})
                    else:
                        response_queue.put(
                            {"id": req_id, "ok": True, "sdk_ok": True, "matrix": fk_mat}
                        )
                    continue

                sp = kine.ik(
                    robot_serial=serial,
                    pose_mat=req["pose_mat"],
                    ref_joints=req["ref_joints"],
                    zsp_type=int(req["zsp_type"]),
                    zsp_para=list(req["zsp_para"]),
                    zsp_angle=float(req["zsp_angle"]),
                    dgr=list(req["dgr"]),
                )
                if sp is False:
                    response_queue.put({"id": req_id, "ok": True, "sdk_ok": False})
                else:
                    response_queue.put(
                        {
                            "id": req_id,
                            "ok": True,
                            "sdk_ok": True,
                            "joints": sp.m_Output_RetJoint.to_list(),
                            "is_out_range": bool(sp.m_Output_IsOutRange),
                            "is_jnt_exd": bool(sp.m_Output_IsJntExd),
                            "jnt_exd_tags": list(sp.m_Output_JntExdTags),
                        }
                    )
            except Exception as exc:
                response_queue.put({"id": req_id, "ok": False, "error": str(exc)})
    finally:
        os.dup2(old_stdout, 1)
        os.dup2(old_stderr, 2)
        os.close(old_stdout)
        os.close(old_stderr)
        os.close(devnull_fd)


class _IkSolveOutput:
    """Minimal stand-in for FX_InvKineSolvePara fields used by teleop."""

    def __init__(
        self,
        joints: list,
        is_out_range: bool,
        is_jnt_exd: bool,
        jnt_exd_tags: list,
    ):
        self.m_Output_RetJoint = SimpleNamespace(to_list=lambda: list(joints))
        self.m_Output_IsOutRange = is_out_range
        self.m_Output_IsJntExd = is_jnt_exd
        self.m_Output_JntExdTags = list(jnt_exd_tags)


class IkSubprocessClient:
    """Persistent spawn child that runs TJ SDK FK/IK."""

    def __init__(
        self,
        config_path: str,
        logger: Any,
        timeout_sec: float = 2.0,
        ready_timeout_sec: float = 45.0,
        max_rate_hz: float = 0.0,
        on_worker_restart=None,
    ):
        self._config_path = config_path
        self._logger = logger
        self._timeout_sec = max(float(timeout_sec), 0.1)
        self._ready_timeout_sec = max(float(ready_timeout_sec), 1.0)
        self._min_interval_sec = (
            0.0 if max_rate_hz <= 0.0 else 1.0 / max(float(max_rate_hz), 1.0)
        )
        self._on_worker_restart = on_worker_restart
        self._last_call_finished = 0.0
        self._ctx = multiprocessing.get_context("spawn")
        self._request_queue: Optional[multiprocessing.Queue] = None
        self._response_queue: Optional[multiprocessing.Queue] = None
        self._process: Optional[multiprocessing.Process] = None
        self._next_id = 0
        self._restart_count = 0
        self._ready = False
        self._busy = False
        self._pnva = None
        self._last_timeout_log = 0.0
        self._start_worker(wait_ready=True)

    @property
    def pnva(self):
        return self._pnva

    def _wait_ready(self) -> bool:
        assert self._response_queue is not None
        assert self._process is not None
        deadline = time.monotonic() + self._ready_timeout_sec
        while time.monotonic() < deadline:
            if not self._process.is_alive():
                code = self._process.exitcode
                self._logger.error(
                    f"IK subprocess worker exited during libKine init (exitcode={code})"
                )
                return False
            try:
                msg = self._response_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg.get("type") == "ready":
                self._ready = True
                self._pnva = msg.get("pnva")
                self._logger.info(
                    f"IK subprocess worker ready (pid={self._process.pid}, "
                    f"init wait <= {self._ready_timeout_sec:.0f}s)"
                )
                if self._restart_count > 1 and self._on_worker_restart is not None:
                    self._on_worker_restart()
                return True
        self._logger.error(
            f"IK subprocess worker not ready within {self._ready_timeout_sec:.0f}s"
        )
        return False

    def _drain_response_queue(self) -> None:
        if self._response_queue is None:
            return
        while True:
            try:
                self._response_queue.get_nowait()
            except queue.Empty:
                break

    def _start_worker(self, wait_ready: bool = True) -> None:
        self.shutdown(wait=False)
        self._ready = False
        self._busy = False
        self._pnva = None
        self._request_queue = self._ctx.Queue(maxsize=8)
        self._response_queue = self._ctx.Queue(maxsize=64)
        self._process = self._ctx.Process(
            target=_ik_worker_loop,
            args=(self._config_path, self._request_queue, self._response_queue),
            name="tianji_ik_worker",
            daemon=True,
        )
        self._process.start()
        self._restart_count += 1
        self._logger.info(
            f"IK subprocess worker starting (pid={self._process.pid}, "
            f"restarts={self._restart_count - 1})..."
        )
        if wait_ready and not self._wait_ready():
            if self._process is not None and self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.0)
            self._process = None

    def _log_worker_died(self) -> None:
        code = None if self._process is None else self._process.exitcode
        self._logger.warning(f"IK subprocess worker died (exitcode={code}); restarting")

    def _ensure_alive(self) -> bool:
        if self._process is not None and self._process.is_alive() and self._ready:
            return True
        if self._process is not None and self._process.is_alive() and not self._ready:
            return self._wait_ready()
        self._log_worker_died()
        self._start_worker(wait_ready=True)
        return self._ready

    def _call(self, payload: dict):
        if not self._ensure_alive():
            return None
        if self._min_interval_sec > 0.0:
            since = time.monotonic() - self._last_call_finished
            if since < self._min_interval_sec:
                return None
        if self._busy:
            return None
        assert self._request_queue is not None
        assert self._response_queue is not None

        self._busy = True
        self._next_id += 1
        req_id = self._next_id
        payload = dict(payload)
        payload["id"] = req_id
        try:
            self._request_queue.put_nowait(payload)
        except queue.Full:
            self._busy = False
            return None

        try:
            deadline = time.monotonic() + self._timeout_sec
            while time.monotonic() < deadline:
                remaining = max(deadline - time.monotonic(), 0.01)
                try:
                    resp = self._response_queue.get(timeout=remaining)
                except queue.Empty:
                    break
                if resp.get("type") == "ready":
                    continue
                if resp.get("id") != req_id:
                    continue
                return resp
        finally:
            self._busy = False
            self._last_call_finished = time.monotonic()

        now = time.monotonic()
        alive = self._process is not None and self._process.is_alive()
        if now - self._last_timeout_log >= 1.0:
            self._last_timeout_log = now
            code = None if self._process is None else self._process.exitcode
            self._logger.warning(
                f"IK subprocess slow (>{self._timeout_sec:.1f}s, alive={alive}, "
                f"exitcode={code}); holding last command"
            )
        if not alive:
            self._ready = False
            self._drain_response_queue()
            self._start_worker(wait_ready=True)
        return None

    def fk(self, robot_serial: int, joints: list):
        resp = self._call(
            {"op": "fk", "robot_serial": robot_serial, "joints": list(joints)}
        )
        if resp is None:
            return False
        if not resp.get("ok"):
            self._logger.warning(f"FK subprocess error: {resp.get('error', 'unknown')}")
            return False
        if not resp.get("sdk_ok"):
            return False
        return resp["matrix"]

    def solve(
        self,
        robot_serial: int,
        pose_mat: list,
        ref_joints: list,
        zsp_type: int,
        zsp_para: list,
        zsp_angle: float,
        dgr: list,
    ):
        resp = self._call(
            {
                "op": "ik",
                "robot_serial": robot_serial,
                "pose_mat": pose_mat,
                "ref_joints": list(ref_joints),
                "zsp_type": zsp_type,
                "zsp_para": list(zsp_para),
                "zsp_angle": float(zsp_angle),
                "dgr": list(dgr),
            }
        )
        if resp is None:
            return False
        if not resp.get("ok"):
            self._logger.warning(f"IK subprocess error: {resp.get('error', 'unknown')}")
            return False
        if not resp.get("sdk_ok"):
            return False
        return _IkSolveOutput(
            joints=resp["joints"],
            is_out_range=resp.get("is_out_range", False),
            is_jnt_exd=resp.get("is_jnt_exd", False),
            jnt_exd_tags=resp.get("jnt_exd_tags", []),
        )

    def restart_worker(self) -> bool:
        """Kill and respawn libKine after a bad IK branch (clears native state)."""
        self._drain_response_queue()
        self._start_worker(wait_ready=True)
        return self._ready

    def shutdown(self, wait: bool = True) -> None:
        if self._request_queue is not None:
            try:
                self._request_queue.put_nowait(None)
            except Exception:
                pass
        if self._process is not None:
            if wait:
                self._process.join(timeout=3.0)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.0)
            self._process = None
        self._request_queue = None
        self._response_queue = None
        self._ready = False
        self._busy = False
