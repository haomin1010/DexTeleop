#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-read}"
COUNT="${2:-50}"
ROBOT_IP="${TIANJI_ROBOT_IP:-192.168.2.166}"

cd /home/user/workspace/DexProj

./scripts/ensure_docker_exec.sh -- bash -lc "
source /workspace/DexProj/scripts/activate_dexproj_env.sh
python3 -u - \"$MODE\" \"$COUNT\" \"$ROBOT_IP\" <<'PY'
import sys
import time

from tianji_output._internal.fx_robot import Marvin_Robot
from tianji_output._internal.structure_data import DCSS

mode = sys.argv[1]
count = int(sys.argv[2])
robot_ip = sys.argv[3]

robot = Marvin_Robot()
ok = robot.connect(robot_ip)
print(f\"[diag] connect {robot_ip}: {ok}\")
if not ok:
    raise SystemExit(2)

def read_right(label=''):
    data = robot.subscribe(DCSS())
    right_state = data.get(\"states\", [{}, {}])[1]
    right_out = data[\"outputs\"][1]
    joints = [float(v) for v in right_out[\"fb_joint_pos\"]]
    cmd = [float(v) for v in right_out.get(\"fb_joint_cmd\", [])]
    vel = [float(v) for v in right_out.get(\"fb_joint_vel\", [])]
    prefix = f\"[diag] {label} \" if label else \"[diag] \"
    print(f\"{prefix}right_state={right_state}\")
    print(f\"{prefix}right joints={joints}\")
    print(f\"{prefix}right cmd={cmd}\")
    print(f\"{prefix}right vel={vel}\")
    try:
        print(f\"{prefix}servo_err_B={robot.get_servo_error_code('B')}\")
    except Exception as exc:
        print(f\"{prefix}servo_err_B unavailable: {exc}\")
    return joints

def read_arm(data, arm, index, label=''):
    state = data.get(\"states\", [{}, {}])[index]
    out = data[\"outputs\"][index]
    joints = [float(v) for v in out[\"fb_joint_pos\"]]
    cmd = [float(v) for v in out.get(\"fb_joint_cmd\", [])]
    vel = [float(v) for v in out.get(\"fb_joint_vel\", [])]
    prefix = f\"[diag] {label} \" if label else \"[diag] \"
    print(f\"{prefix}{arm}_state={state}\")
    print(f\"{prefix}{arm} joints={joints}\")
    print(f\"{prefix}{arm} cmd={cmd}\")
    print(f\"{prefix}{arm} vel={vel}\")
    try:
        print(f\"{prefix}servo_err_{arm}={robot.get_servo_error_code(arm)}\")
    except Exception as exc:
        print(f\"{prefix}servo_err_{arm} unavailable: {exc}\")
    return joints

def read_both(label=''):
    data = robot.subscribe(DCSS())
    return {
        \"A\": read_arm(data, \"A\", 0, label),
        \"B\": read_arm(data, \"B\", 1, label),
    }

def disable_right():
    robot.clear_set()
    robot.set_state(arm=\"A\", state=0)
    robot.set_state(arm=\"B\", state=0)
    robot.send_cmd()
    print(\"[diag] sent A/B state=0\")

try:
    joints = read_right(\"initial\")
    if mode == \"read\":
        raise SystemExit(0)

    if mode not in {\"enable_disable\", \"hold_once\", \"hold_loop\", \"jog_small\", \"jog_position\", \"jog_position_tj\", \"jog_state3_tj\", \"move_right_init_tj\"}:
        raise SystemExit(f\"unknown mode: {mode}\")

    if mode in {\"jog_position_tj\", \"jog_state3_tj\", \"move_right_init_tj\"}:
        command_state = 3
        print(f\"[diag] using TJ-style dual-arm state={command_state} entry\")
        read_both(\"tj_initial\")

        robot.clear_set()
        robot.clear_error(\"A\")
        robot.clear_error(\"B\")
        robot.send_cmd()
        time.sleep(0.5)

        robot.clear_set()
        robot.set_state(arm=\"A\", state=0)
        robot.set_state(arm=\"B\", state=0)
        robot.send_cmd()
        print(\"[diag] TJ sent A/B state=0\")
        time.sleep(0.5)

        robot.clear_set()
        robot.clear_error(\"A\")
        robot.clear_error(\"B\")
        robot.send_cmd()
        time.sleep(0.5)

        joints_by_arm = read_both(\"tj_before_enable\")
        command_hold_by_arm = {
            \"A\": list(joints_by_arm[\"A\"]),
            \"B\": list(joints_by_arm[\"B\"]),
        }
        robot.clear_set()
        for arm in (\"A\", \"B\"):
            robot.set_state(arm=arm, state=command_state)
            robot.set_vel_acc(arm=arm, velRatio=20, AccRatio=10)
            robot.set_joint_cmd_pose(arm=arm, joints=command_hold_by_arm[arm])
        robot.send_cmd()
        print(f\"[diag] TJ sent A/B state={command_state} vel=20 acc=10 plus current-joint hold\")
        time.sleep(0.5)
        if command_state == 3:
            joint_k = [2, 2, 2, 1.6, 1, 1, 1]
            joint_d = [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]
            robot.clear_set()
            robot.set_impedance_type(arm=\"A\", type=1)
            robot.set_impedance_type(arm=\"B\", type=1)
            robot.set_joint_kd_params(arm=\"A\", K=joint_k, D=joint_d)
            robot.set_joint_kd_params(arm=\"B\", K=joint_k, D=joint_d)
            robot.set_drag_space(arm=\"A\", dgType=0)
            robot.set_drag_space(arm=\"B\", dgType=0)
            robot.send_cmd()
            print(\"[diag] TJ sent A/B impedance_type=1 joint_kd drag_space=0\")
        time.sleep(1.0)
        read_both(\"tj_after_enable\")

        start_b = list(command_hold_by_arm[\"B\"])
        if mode == \"move_right_init_tj\":
            target_b = [-90.0, -90.0, 90.0, -90.0, 0.0, 0.0, 0.0]
            print(f\"[diag] moving B to init target: {target_b}\")
        else:
            target_b = list(start_b)
            target_b[0] += 1.0
        hold_a = list(command_hold_by_arm[\"A\"])
        for index in range(count):
            ratio = (index + 1) / max(count, 1)
            cmd_b = [start_b[i] + ratio * (target_b[i] - start_b[i]) for i in range(7)]
            robot.clear_set()
            robot.set_joint_cmd_pose(arm=\"A\", joints=hold_a)
            robot.set_joint_cmd_pose(arm=\"B\", joints=cmd_b)
            robot.send_cmd()
            if index % 10 == 0:
                action = \"move B init\" if mode == \"move_right_init_tj\" else \"jog B out\"
                print(f\"[diag] TJ {action} {index + 1}/{count}: {cmd_b}\")
            time.sleep(0.02)
        if mode == \"move_right_init_tj\":
            read_both(\"tj_after_move_init\")
            raise SystemExit(0)
        for index in range(count):
            ratio = (index + 1) / max(count, 1)
            cmd_b = [target_b[i] + ratio * (start_b[i] - target_b[i]) for i in range(7)]
            robot.clear_set()
            robot.set_joint_cmd_pose(arm=\"A\", joints=hold_a)
            robot.set_joint_cmd_pose(arm=\"B\", joints=cmd_b)
            robot.send_cmd()
            if index % 10 == 0:
                print(f\"[diag] TJ jog B back {index + 1}/{count}: {cmd_b}\")
            time.sleep(0.02)
        read_both(\"tj_after_jog\")
        raise SystemExit(0)

    robot.clear_set()
    robot.clear_error(\"B\")
    command_state = 3
    robot.set_state(arm=\"B\", state=command_state)
    robot.set_vel_acc(arm=\"B\", velRatio=20, AccRatio=10)
    robot.send_cmd()
    print(f\"[diag] sent B state={command_state} vel=20 acc=10\")
    time.sleep(0.5)
    joints = read_right(\"after_enable\")

    if mode == \"hold_once\":
        robot.clear_set()
        robot.set_joint_cmd_pose(arm=\"B\", joints=joints)
        robot.send_cmd()
        print(f\"[diag] sent one hold command: {joints}\")
        time.sleep(0.5)
        read_right(\"after_hold_once\")

    if mode == \"hold_loop\":
        for index in range(count):
            robot.clear_set()
            robot.set_joint_cmd_pose(arm=\"B\", joints=joints)
            robot.send_cmd()
            if index % 10 == 0:
                print(f\"[diag] hold_loop {index + 1}/{count}\")
            time.sleep(0.02)
        read_right(\"after_hold_loop\")

    if mode in {\"jog_small\", \"jog_position\"}:
        target = list(joints)
        target[0] += 1.0
        for index in range(count):
            ratio = (index + 1) / max(count, 1)
            cmd = [joints[i] + ratio * (target[i] - joints[i]) for i in range(7)]
            robot.clear_set()
            robot.set_joint_cmd_pose(arm=\"B\", joints=cmd)
            robot.send_cmd()
            if index % 10 == 0:
                print(f\"[diag] jog_small out {index + 1}/{count}: {cmd}\")
            time.sleep(0.02)
        for index in range(count):
            ratio = (index + 1) / max(count, 1)
            cmd = [target[i] + ratio * (joints[i] - target[i]) for i in range(7)]
            robot.clear_set()
            robot.set_joint_cmd_pose(arm=\"B\", joints=cmd)
            robot.send_cmd()
            if index % 10 == 0:
                print(f\"[diag] jog_small back {index + 1}/{count}: {cmd}\")
            time.sleep(0.02)
        read_right(\"after_jog\")
finally:
    if mode != \"read\":
        try:
            disable_right()
            time.sleep(0.5)
            read_right(\"after_disable\")
        except Exception as exc:
            print(f\"[diag] disable failed: {exc}\")
    robot.release_robot()
    print(\"[diag] released robot\")
PY
"
