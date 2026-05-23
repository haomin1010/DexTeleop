# DexProj Implementation Notes

## Current Local Entry Points

HTC device check:

```bash
scripts/check_devices.sh
```

HTC bringup command resolution:

```bash
scripts/bringup_teleop.sh --dry-run --skip-preflight
```

Unified session plan resolution:

```bash
scripts/run_session.sh --dry-run --skip-preflight
scripts/run_session.sh --json --skip-preflight
```

Simulated runtime without spawning real child processes:

```bash
scripts/run_session.sh --simulate-runtime --skip-preflight --startup-wait-sec 0
```

Real runtime entrypoint:

```bash
scripts/run_session.sh --skip-preflight
```

## What Exists Now

The current implementation already gives us:

- a local `dexproj` Python package
- HTC/OpenVR tracker config owned by DexProj
- device-check command
- HTC-only bringup wrapper
- hand teleop config model with explicit left/right glove and hand serial numbers
- unified session config and plan output
- real subprocess plan generation for:
  - `wuji-hand-teleop` HTC bringup
  - `wuji-retargeting/example/teleop_real.py` left/right hand teleop
- runtime state machine: `initialized -> ready -> running -> stopped`
- trigger abstraction with three modes:
  - `keyboard`
  - `gamepad`
  - `both`
- keyboard trigger path:
  - start: `B`
  - stop: `E`
- gamepad trigger design aligned with TJ defaults:
  - start: `LB/RB`
  - stop: `START`
- automatic fallback from `both` to keyboard when Python gamepad backend is unavailable
- minimal session recorder that creates `data/raw/session_YYYY_MM_DD/episode_xxxxxx/`
- `meta.json` output with mode, trigger, bringup, hand config, process plan, and runtime timestamps

## Important Runtime Behavior

Current `run_session` behavior is:

1. Resolve DexProj config
2. Optionally run preflight
3. Build HTC bringup command
4. Build left/right hand teleop subprocess commands
5. Spawn subprocesses unless `--simulate-runtime` is used
6. Enter `ready`
7. Wait for start trigger
8. Create episode directory and `meta.json`
9. Wait for stop trigger
10. Stop all managed subprocesses and finalize `meta.json`

## Current Limitations

The current implementation does not yet provide:

- confirmed production-ready gamepad backend installation in the environment
- stream capture for arm, hand, or camera data payloads
- camera subprocess orchestration inside the unified session runner
- session health checks beyond child-process exit detection
- per-process log redirection into episode directories
- dataset export and post-processing

## Next Layer

The next implementation layer should focus on:

- writing live arm/hand/camera streams into each episode directory
- attaching per-process stdout/stderr logs into episode runtime artifacts
- integrating a production gamepad backend in the target runtime environment
- adding camera bringup and recorder wiring
- aligning `meta.json` and future dataset export schema
