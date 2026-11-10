#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from typing import Any

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.damiao import DamiaoMotorsBus
from lerobot.motors.robstride import RobstrideMotorsBus
from lerobot.processor import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_hyopenarm_leader import HyOpenArmLeaderConfig

logger = logging.getLogger(__name__)


class HyOpenArmLeader(Teleoperator):
    """Leader arm supporting Robstride, Damiao, or mixed motor configurations.

    Motors are routed to their configured bus, allowing all-Damiao,
    all-Robstride, or mixed-motor arms.
    """

    config_class = HyOpenArmLeaderConfig
    name = "hyopenarm_leader"

    def __init__(self, config: HyOpenArmLeaderConfig):
        super().__init__(config)
        self.config = config

        robstride_motors: dict[str, Motor] = {}
        damiao_motors: dict[str, Motor] = {}
        self.motor_to_bus_name: dict[str, str] = {}

        for motor_name, (send_id, recv_id, motor_type_str, motor_bus) in config.motor_config.items():
            motor = Motor(send_id, motor_type_str, MotorNormMode.DEGREES)
            motor.recv_id = recv_id
            motor.motor_type_str = motor_type_str

            if motor_bus == "robstride":
                robstride_motors[motor_name] = motor
            elif motor_bus == "damiao":
                damiao_motors[motor_name] = motor
            else:
                raise ValueError(f"Unsupported motor bus '{motor_bus}' for motor '{motor_name}'")

            self.motor_to_bus_name[motor_name] = motor_bus

        self.robstride_bus = RobstrideMotorsBus(
            port=self.config.port,
            motors=robstride_motors,
            calibration={k: v for k, v in self.calibration.items() if k in robstride_motors},
            can_interface=self.config.can_interface,
            use_can_fd=self.config.use_can_fd,
            bitrate=self.config.can_bitrate,
            data_bitrate=self.config.can_data_bitrate if self.config.use_can_fd else None,
        )
        self.damiao_bus = DamiaoMotorsBus(
            port=self.config.port,
            motors=damiao_motors,
            calibration={k: v for k, v in self.calibration.items() if k in damiao_motors},
            can_interface=self.config.can_interface,
            use_can_fd=self.config.use_can_fd,
            bitrate=self.config.can_bitrate,
            data_bitrate=self.config.can_data_bitrate if self.config.use_can_fd else None,
        )

        self.buses = {
            "robstride": self.robstride_bus,
            "damiao": self.damiao_bus,
        }
        self.active_buses = {name: bus for name, bus in self.buses.items() if bus.motors}

    @property
    def action_features(self) -> dict[str, type]:
        features: dict[str, type] = {}
        for motor_name in self.config.motor_config:
            features[f"{motor_name}.pos"] = float
            features[f"{motor_name}.vel"] = float
            features[f"{motor_name}.torque"] = float
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return all(bus.is_connected for bus in self.active_buses.values())

    @property
    def is_calibrated(self) -> bool:
        return all(bus.is_calibrated for bus in self.active_buses.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        for bus_name, bus in self.active_buses.items():
            logger.info(f"Connecting {bus_name.title()} arm bus on {self.config.port}...")
            bus.connect()

        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                for bus in self.active_buses.values():
                    bus.write_calibration({k: v for k, v in self.calibration.items() if k in bus.motors})
                return

        logger.info(f"\nRunning calibration for {self}")
        for bus in self.active_buses.values():
            bus.disable_torque()

        input(
            "\nCalibration: Set Zero Position\n"
            "Position the arm in the reference configuration, then press ENTER..."
        )

        for bus in self.active_buses.values():
            bus.set_zero_position()
        logger.info("Arm zero position set.")

        logger.info("Setting default range: -90° to +90° for all joints")
        for bus in self.active_buses.values():
            for motor_name, motor in bus.motors.items():
                self.calibration[motor_name] = MotorCalibration(
                    id=motor.id,
                    drive_mode=0,
                    homing_offset=0,
                    range_min=-90,
                    range_max=90,
                )

        for bus in self.active_buses.values():
            bus.write_calibration({k: v for k, v in self.calibration.items() if k in bus.motors})
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        if self.config.manual_control:
            for bus in self.active_buses.values():
                bus.disable_torque()
            return

        for bus in self.active_buses.values():
            bus.configure_motors()

        for idx, motor_name in enumerate(self.config.motor_config.keys()):
            kp = self.config.position_kp[idx]
            kd = self.config.position_kd[idx]
            bus = self._get_bus_for_motor(motor_name)
            bus.write("Kp", motor_name, kp)
            bus.write("Kd", motor_name, kd)

    def setup_motors(self) -> None:
        raise NotImplementedError(
            "Motor ID configuration is typically done via manufacturer tools for CAN motors."
        )

    def _get_bus_for_motor(self, motor_name: str):
        bus_name = self.motor_to_bus_name[motor_name]
        return self.buses[bus_name]

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        start = time.perf_counter()
        action_dict: dict[str, Any] = {}

        all_states = {}
        for bus in self.active_buses.values():
            all_states.update(bus.sync_read_all_states())

        for motor_name in self.config.motor_config:
            state = all_states.get(motor_name, {})
            action_dict[f"{motor_name}.pos"] = state.get("position")
            # pos = state.get("position")
            # if motor_name == "joint_4" and pos is not None:
            #     logger.warning(f"[DEBUG] joint_4 before flip = {pos}")
            #     pos = -pos
            #     logger.warning(f"[DEBUG] joint_4 after  flip = {pos}")        
            # action_dict[f"{motor_name}.pos"] = pos
            action_dict[f"{motor_name}.vel"] = state.get("velocity")
            action_dict[f"{motor_name}.torque"] = state.get("torque")

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")
        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError("Feedback is not yet implemented for HyOpenArm leader.")

    @check_if_not_connected
    def disconnect(self) -> None:
        for bus in self.active_buses.values():
            bus.disconnect(disable_torque=self.config.manual_control)
        logger.info(f"{self} disconnected.")
