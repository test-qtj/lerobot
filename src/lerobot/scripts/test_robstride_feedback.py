#!/usr/bin/env python3

import argparse
import logging
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.robstride.robstride import RobstrideMotorsBus


logging.basicConfig(
    level=logging.WARNING,
    format="%(levelname)s %(asctime)s %(name)s:%(lineno)d %(message)s",
)


def make_motor(send_id: int, recv_id: int, motor_type: str) -> Motor:
    motor = Motor(send_id, motor_type, MotorNormMode.DEGREES)
    motor.recv_id = recv_id
    motor.motor_type_str = motor_type
    return motor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="can0", help="CAN interface, e.g. can0 or can1")
    parser.add_argument("--use_can_fd", action="store_true", help="Use CAN FD")
    args = parser.parse_args()

    motors = {
        "joint_1": make_motor(1, 1, "O6"),
        "joint_2": make_motor(2, 2, "O6"),
        "joint_3": make_motor(3, 3, "O0"),
        "joint_4": make_motor(4, 4, "O0"),
    }

    bus = RobstrideMotorsBus(
        port=args.port,
        motors=motors,
        can_interface="socketcan",
        use_can_fd=args.use_can_fd,
        bitrate=1_000_000,
        data_bitrate=5_000_000 if args.use_can_fd else None,
    )

    print(f"Connecting Robstride bus on {args.port}, use_can_fd={args.use_can_fd}")
    bus.connect(handshake=True)

    print("Start reading states...")
    try:
        for i in range(50):
            print(f"\n===== read round {i} =====")
            states = bus.sync_read_all_states(["joint_1", "joint_2", "joint_3", "joint_4"])
            for name, state in states.items():
                print(name, state)
            time.sleep(0.05)
    finally:
        bus.disconnect(disable_torque=True)
        print("Disconnected.")


if __name__ == "__main__":
    main()