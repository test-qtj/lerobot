#!/usr/bin/env python3
import time
import argparse
import can


MOTORS = {
    "joint_1": (0x001, 0x011),
    "joint_2": (0x002, 0x012),
    "joint_3": (0x003, 0x013),
    "joint_4": (0x004, 0x014),
    "joint_5": (0x005, 0x015),
    "joint_6": (0x006, 0x016),
    "joint_7": (0x007, 0x017),
    "gripper": (0x008, 0x018),
}

ENABLE_CMD = bytes.fromhex("FF FF FF FF FF FF FF FC")
DISABLE_CMD = bytes.fromhex("FF FF FF FF FF FF FF FD")


def open_bus(channel, use_fd=True, bitrate=1000000, data_bitrate=5000000):
    """
    按 LeRobot damiao.py 的 socketcan 方式打开总线。
    如果系统已经用 ip link 配置好了 can0/can1，这里的 bitrate/fd 参数通常不会重新配置内核接口，
    但 python-can 会知道这是 fd bus。
    """
    kwargs = {
        "channel": channel,
        "interface": "socketcan",
        "bitrate": bitrate,
    }

    if use_fd:
        kwargs.update({
            "fd": True,
            "data_bitrate": data_bitrate,
        })

    return can.interface.Bus(**kwargs)


def send_cmd(bus, can_id, data, use_fd=True):
    """
    按 damiao.py 方式发送：
    can.Message(..., is_fd=self.use_can_fd)
    """
    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=False,
        is_fd=use_fd,
    )
    bus.send(msg)


def drain_bus(bus, max_time=0.02):
    """
    类似 damiao.py 握手前清空旧消息：
    while self.canbus.recv(timeout=0.01): pass

    这里加 max_time，避免总线上一直有数据导致死循环。
    """
    start = time.time()
    while time.time() - start < max_time:
        msg = bus.recv(timeout=0.001)
        if msg is None:
            break


def wait_expected_response(bus, expected_id, timeout=0.1):
    """
    按 damiao.py 的握手逻辑：
    在 timeout 时间内反复 recv；
    只要收到 arbitration_id == expected_id，就认为该电机有回传。
    同时记录窗口内看到的所有 ID，方便调试。
    """
    start_time = time.time()
    seen_ids = []

    while time.time() - start_time < timeout:
        remaining = timeout - (time.time() - start_time)
        recv_timeout = min(0.01, max(0.0001, remaining))

        msg = bus.recv(timeout=recv_timeout)
        if msg is None:
            continue

        seen_ids.append(msg.arbitration_id)

        if msg.arbitration_id == expected_id:
            return True, seen_ids

    return False, seen_ids


def test_one_motor(bus, motor_name, send_id, feedback_id, args):
    """
    按 damiao.py 的 handshake 风格测试一个电机：

    1. 清空旧消息；
    2. 发 enable；
    3. 在 feedback_window 内等 feedback_id；
    4. 可选：再发 disable，同样等 feedback_id；
    5. 如果 enable 或 disable 任意一次收到 feedback_id，则本轮成功。
    """
    enable_ok = False
    disable_ok = False
    enable_seen = []
    disable_seen = []

    # 每个电机测试前清空旧消息，避免上一个电机的回传影响当前判断
    if args.drain:
        drain_bus(bus, max_time=args.drain_time)

    # enable
    try:
        send_cmd(bus, send_id, ENABLE_CMD, use_fd=args.fd)
        enable_ok, enable_seen = wait_expected_response(
            bus,
            expected_id=feedback_id,
            timeout=args.feedback_window,
        )
    except can.CanError as e:
        print(f"{motor_name}: enable 发送失败: {e}")
        enable_ok = False
        enable_seen = []

    time.sleep(args.command_gap)

    # disable
    if args.send_disable:
        try:
            send_cmd(bus, send_id, DISABLE_CMD, use_fd=args.fd)
            disable_ok, disable_seen = wait_expected_response(
                bus,
                expected_id=feedback_id,
                timeout=args.feedback_window,
            )
        except can.CanError as e:
            print(f"{motor_name}: disable 发送失败: {e}")
            disable_ok = False
            disable_seen = []
    else:
        disable_ok = False
        disable_seen = []

    round_ok = enable_ok or disable_ok
    all_seen = enable_seen + disable_seen

    return round_ok, enable_ok, disable_ok, all_seen


def make_stats(motors):
    stats = {}

    for name, (send_id, feedback_id) in motors.items():
        stats[name] = {
            "send_id": send_id,
            "feedback_id": feedback_id,

            "tests": 0,
            "ok": 0,
            "fail": 0,

            "enable_fail": 0,
            "disable_fail": 0,

            "seen_ids": set(),
        }

    return stats


def update_stats(stats, motor_name, round_ok, enable_ok, disable_ok, seen_ids, send_disable=True):
    s = stats[motor_name]

    s["tests"] += 1
    s["seen_ids"].update(seen_ids)

    if round_ok:
        s["ok"] += 1
    else:
        s["fail"] += 1

    if not enable_ok:
        s["enable_fail"] += 1

    if send_disable and not disable_ok:
        s["disable_fail"] += 1


def percent(a, b):
    if b == 0:
        return 0.0
    return 100.0 * a / b


def print_one_arm_result(arm_name, channel, stats):
    print()
    print("=" * 72)
    print(f"{arm_name} {channel} 统计结果")
    print("=" * 72)

    for name, s in stats.items():
        tests = s["tests"]
        ok = s["ok"]
        fail = s["fail"]

        if tests == 0:
            print(f"{name}: 未测试")
            continue

        fail_rate = percent(fail, tests)

        if fail == 0:
            status = "正常"
        elif fail == tests:
            status = "完全无回传"
        else:
            status = "回传不稳定"

        print(
            f"{name}: {status} | "
            f"测试 {tests} 次，成功 {ok} 次，失败 {fail} 次，失败率 {fail_rate:.1f}%"
        )

        if fail > 0:
            seen = sorted(s["seen_ids"])
            seen_str = ", ".join(f"0x{x:03X}" for x in seen) if seen else "无"

            print(
                f"    期望回传 0x{s['feedback_id']:03X}，"
                f"使能失败 {s['enable_fail']} 次，"
                f"失能失败 {s['disable_fail']} 次，"
                f"测试期间读到过的 ID: {seen_str}"
            )

    print("=" * 72)


def print_final_judgement(arm_name, stats):
    no_response = []
    unstable = []

    for name, s in stats.items():
        tests = s["tests"]
        fail = s["fail"]

        if tests == 0:
            continue

        if fail == tests:
            no_response.append(name)
        elif fail > 0:
            unstable.append((name, fail, tests, percent(fail, tests)))

    print()
    print(f"[{arm_name}] 最终判断：")

    if not no_response and not unstable:
        print("  所有电机回传正常。")
        return

    if no_response:
        print("  完全无回传的电机：")
        for name in no_response:
            print(f"    - {name}")

    if unstable:
        print("  有回传但不稳定的电机：")
        for name, fail, tests, fail_rate in unstable:
            print(f"    - {name}: 失败 {fail}/{tests} 次，失败率 {fail_rate:.1f}%")

    if no_response:
        print("  优先查这些电机的供电、CANH/CANL、ID、是否接在对应总线上。")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--left-channel", default="can0")
    parser.add_argument("--right-channel", default="can1")

    parser.add_argument("--duration", type=float, default=60.0)

    # LeRobot damiao.py handshake 是 0.1s，这里默认也用 0.1s
    parser.add_argument("--feedback-window", type=float, default=0.1)

    parser.add_argument("--command-gap", type=float, default=0.01)
    parser.add_argument("--motor-gap", type=float, default=0.01)
    parser.add_argument("--summary-interval", type=float, default=20.0)

    # LeRobot OpenArm 默认 use_can_fd=True，所以这里默认也建议用 FD。
    # 如果你要发 classic CAN 帧，加 --no-fd。
    parser.add_argument("--fd", dest="fd", action="store_true", default=True)
    parser.add_argument("--no-fd", dest="fd", action="store_false")

    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--data-bitrate", type=int, default=5000000)

    parser.add_argument("--only", default="")

    # 是否发送 disable。默认发送，因为你想测试 enable/disable 回传。
    parser.add_argument("--send-disable", action="store_true", default=True)
    parser.add_argument("--no-disable", dest="send_disable", action="store_false")

    # 是否每个电机测试前清旧消息。默认开启，模拟 damiao.py handshake。
    parser.add_argument("--drain", action="store_true", default=True)
    parser.add_argument("--no-drain", dest="drain", action="store_false")
    parser.add_argument("--drain-time", type=float, default=0.02)

    args = parser.parse_args()

    motors = MOTORS.copy()

    if args.only.strip():
        wanted = {x.strip() for x in args.only.split(",") if x.strip()}
        motors = {name: ids for name, ids in motors.items() if name in wanted}

        if not motors:
            raise ValueError(f"--only={args.only} 没有匹配到有效电机")

    print()
    print("双臂 SocketCAN 回传测试：LeRobot damiao.py 风格")
    print(f"左臂接口: {args.left_channel}")
    print(f"右臂接口: {args.right_channel}")
    print(f"总测试时间: {args.duration} 秒")
    print(f"每次命令后等待回传: {args.feedback_window} 秒")
    print(f"发送 CAN FD 帧: {args.fd}")
    print(f"bitrate: {args.bitrate}")
    print(f"data_bitrate: {args.data_bitrate}")
    print(f"每轮发送 disable: {args.send_disable}")
    print(f"每个电机前清空旧消息: {args.drain}")
    print()

    print("电机映射：")
    for name, (send_id, feedback_id) in motors.items():
        print(f"  {name}: 发送 0x{send_id:03X} -> 期望回传 0x{feedback_id:03X}")

    print()
    print("开始测试。只在某一轮无目标回传时打印提示。")
    print()

    left_bus = open_bus(
        args.left_channel,
        use_fd=args.fd,
        bitrate=args.bitrate,
        data_bitrate=args.data_bitrate,
    )
    right_bus = open_bus(
        args.right_channel,
        use_fd=args.fd,
        bitrate=args.bitrate,
        data_bitrate=args.data_bitrate,
    )

    left_stats = make_stats(motors)
    right_stats = make_stats(motors)

    start = time.time()
    next_summary = start + args.summary_interval

    try:
        while True:
            if time.time() - start >= args.duration:
                break

            for motor_name, (send_id, feedback_id) in motors.items():
                if time.time() - start >= args.duration:
                    break

                # 左臂 can0
                left_round_ok, left_enable_ok, left_disable_ok, left_seen = test_one_motor(
                    left_bus,
                    motor_name,
                    send_id,
                    feedback_id,
                    args,
                )

                update_stats(
                    left_stats,
                    motor_name,
                    left_round_ok,
                    left_enable_ok,
                    left_disable_ok,
                    left_seen,
                    send_disable=args.send_disable,
                )

                if not left_round_ok:
                    seen_str = ", ".join(f"0x{x:03X}" for x in sorted(set(left_seen))) if left_seen else "无"
                    print(
                        f"左臂 {motor_name}: 本轮无目标回传 "
                        f"(期望 0x{feedback_id:03X}，本轮读到 ID: {seen_str})"
                    )

                # 右臂 can1
                right_round_ok, right_enable_ok, right_disable_ok, right_seen = test_one_motor(
                    right_bus,
                    motor_name,
                    send_id,
                    feedback_id,
                    args,
                )

                update_stats(
                    right_stats,
                    motor_name,
                    right_round_ok,
                    right_enable_ok,
                    right_disable_ok,
                    right_seen,
                    send_disable=args.send_disable,
                )

                if not right_round_ok:
                    seen_str = ", ".join(f"0x{x:03X}" for x in sorted(set(right_seen))) if right_seen else "无"
                    print(
                        f"右臂 {motor_name}: 本轮无目标回传 "
                        f"(期望 0x{feedback_id:03X}，本轮读到 ID: {seen_str})"
                    )

                time.sleep(args.motor_gap)

            if time.time() >= next_summary:
                elapsed = time.time() - start
                print_one_arm_result(
                    f"左臂 当前统计，用时 {elapsed:.1f} 秒",
                    args.left_channel,
                    left_stats,
                )
                print_one_arm_result(
                    f"右臂 当前统计，用时 {elapsed:.1f} 秒",
                    args.right_channel,
                    right_stats,
                )

                next_summary = time.time() + args.summary_interval

    except KeyboardInterrupt:
        print()
        print("用户中断测试，输出当前结果。")

    finally:
        left_bus.shutdown()
        right_bus.shutdown()

    elapsed = time.time() - start

    print_one_arm_result(
        f"左臂 最终统计，总用时 {elapsed:.1f} 秒",
        args.left_channel,
        left_stats,
    )

    print_one_arm_result(
        f"右臂 最终统计，总用时 {elapsed:.1f} 秒",
        args.right_channel,
        right_stats,
    )

    print_final_judgement("左臂", left_stats)
    print_final_judgement("右臂", right_stats)


if __name__ == "__main__":
    main()