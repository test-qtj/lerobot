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
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.damiao import DamiaoMotorsBus
from lerobot.motors.robstride import RobstrideMotorsBus
from lerobot.processor import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_hyopenarm_follower import (
    LEFT_DEFAULT_JOINTS_LIMITS,
    RIGHT_DEFAULT_JOINTS_LIMITS,
    HyOpenArmFollowerConfig,
)

logger = logging.getLogger(__name__)


class HyOpenArmFollower(Robot):
    """Hybrid follower robot using Robstride + Damiao buses in leader-style composition.

    Motor split:
    - joint_1 ~ joint_4: Robstride
    - joint_5 ~ gripper: Damiao DM4310
    """

    config_class = HyOpenArmFollowerConfig
    name = "hyopenarm_follower"

    def __init__(self, config: HyOpenArmFollowerConfig):
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

        if config.side is not None:
            if config.side == "left":
                config.joint_limits = LEFT_DEFAULT_JOINTS_LIMITS
            elif config.side == "right":
                config.joint_limits = RIGHT_DEFAULT_JOINTS_LIMITS
            else:
                raise ValueError(
                    "config.side must be either 'left', 'right' (for default values) or 'None' (for CLI values)"
                )
        else:
            logger.info(
                "Set config.side to either 'left' or 'right' to use pre-configured values for joint limits."
            )
        logger.info(f"Values used for joint limits: {config.joint_limits}.")

        self.cameras = make_cameras_from_configs(config.cameras)

    def _get_bus_for_motor(self, motor_name: str):
        return self.buses[self.motor_to_bus_name[motor_name]]

    @property
    def _motors_ft(self) -> dict[str, type]:
        features: dict[str, type] = {}
        for motor_name in self.config.motor_config:
            features[f"{motor_name}.pos"] = float
            features[f"{motor_name}.vel"] = float
            features[f"{motor_name}.torque"] = float
        return features

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        buses_connected = self.robstride_bus.is_connected and self.damiao_bus.is_connected
        cameras_connected = all(cam.is_connected for cam in self.cameras.values())
        return buses_connected and cameras_connected

    @property
    def is_calibrated(self) -> bool:
        return self.robstride_bus.is_calibrated and self.damiao_bus.is_calibrated

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting Robstride arm bus on {self.config.port}...")
        self.robstride_bus.connect()

        logger.info(f"Connecting Damiao arm bus on {self.config.port}...")
        self.damiao_bus.connect()

        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        self.robstride_bus.enable_torque()
        self.damiao_bus.enable_torque()
        logger.info(f"{self} connected.")

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.robstride_bus.write_calibration(
                    {k: v for k, v in self.calibration.items() if k in self.robstride_bus.motors}
                )
                self.damiao_bus.write_calibration(
                    {k: v for k, v in self.calibration.items() if k in self.damiao_bus.motors}
                )
                return

        logger.info(f"\nRunning calibration for {self}")
        self.robstride_bus.disable_torque()
        self.damiao_bus.disable_torque()

        input(
            "\nCalibration: Set Zero Position\n"
            "Position the arm in the reference configuration, then press ENTER..."
        )

        self.robstride_bus.set_zero_position()
        self.damiao_bus.set_zero_position()
        logger.info("Hybrid arm zero position set.")

        logger.info("Setting range: -90° to +90° for safety by default for all joints")
        for bus in (self.robstride_bus, self.damiao_bus):
            for motor_name, motor in bus.motors.items():
                self.calibration[motor_name] = MotorCalibration(
                    id=motor.id,
                    drive_mode=0,
                    homing_offset=0,
                    range_min=-90,
                    range_max=90,
                )

        self.robstride_bus.write_calibration(
            {k: v for k, v in self.calibration.items() if k in self.robstride_bus.motors}
        )
        self.damiao_bus.write_calibration(
            {k: v for k, v in self.calibration.items() if k in self.damiao_bus.motors}
        )
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.robstride_bus.configure_motors()
        self.damiao_bus.configure_motors()

        for idx, motor_name in enumerate(self.config.motor_config.keys()):
            bus = self._get_bus_for_motor(motor_name)
            kp = self.config.position_kp[idx]
            kd = self.config.position_kd[idx]
            bus.write("Kp", motor_name, kp)
            bus.write("Kd", motor_name, kd)

    def setup_motors(self) -> None:
        raise NotImplementedError(
            "Motor ID configuration is typically done via manufacturer tools for CAN motors."
        )

    def _read_all_states(self) -> dict[str, dict[str, float]]:
        all_states: dict[str, dict[str, float]] = {}
        if self.robstride_bus.motors:
            all_states.update(self.robstride_bus.sync_read_all_states())
        if self.damiao_bus.motors:
            all_states.update(self.damiao_bus.sync_read_all_states())
        return all_states

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()
        obs_dict: dict[str, Any] = {}

        all_states = self._read_all_states()
        for motor_name in self.config.motor_config:
            state = all_states.get(motor_name, {})
            obs_dict[f"{motor_name}.pos"] = state.get("position", 0.0)
            obs_dict[f"{motor_name}.vel"] = state.get("velocity", 0.0)
            obs_dict[f"{motor_name}.torque"] = state.get("torque", 0.0)

        for cam_key, cam in self.cameras.items():
            cam_start = time.perf_counter()
            obs_dict[cam_key] = cam.read_latest()
            logger.debug(f"{self} read {cam_key}: {(time.perf_counter() - cam_start) * 1e3:.1f}ms")

        logger.debug(f"{self} get_observation took: {(time.perf_counter() - start) * 1e3:.1f}ms")
        return obs_dict

    @check_if_not_connected
    def send_action(
        self,
        action: RobotAction,
        custom_kp: dict[str, float] | None = None,
        custom_kd: dict[str, float] | None = None,
    ) -> RobotAction:
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        for motor_name, position in list(goal_pos.items()):
            if motor_name in self.config.joint_limits:
                min_limit, max_limit = self.config.joint_limits[motor_name]
                clipped_position = max(min_limit, min(max_limit, position))
                if clipped_position != position:
                    logger.debug(f"Clipped {motor_name} from {position:.2f}° to {clipped_position:.2f}°")
                goal_pos[motor_name] = clipped_position

        if self.config.max_relative_target is not None and goal_pos:
            all_states = self._read_all_states()
            present_pos = {
                motor_name: all_states.get(motor_name, {}).get("position", 0.0) for motor_name in goal_pos
            }
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        motor_index = {motor_name: idx for idx, motor_name in enumerate(self.config.motor_config.keys())}
        bus_commands: dict[str, dict[str, tuple[float, float, float, float, float]]] = {
            "robstride": {},
            "damiao": {},
        }

        for motor_name, position_degrees in goal_pos.items():
            idx = motor_index[motor_name]
            kp = custom_kp[motor_name] if custom_kp and motor_name in custom_kp else self.config.position_kp[idx]
            kd = custom_kd[motor_name] if custom_kd and motor_name in custom_kd else self.config.position_kd[idx]
            bus_name = self.motor_to_bus_name[motor_name]
            bus_commands[bus_name][motor_name] = (kp, kd, position_degrees, 0.0, 0.0)

        if bus_commands["robstride"]:
            self.robstride_bus._mit_control_batch(bus_commands["robstride"])
        if bus_commands["damiao"]:
            self.damiao_bus._mit_control_batch(bus_commands["damiao"])

        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    @check_if_not_connected
    def disconnect(self) -> None:
        self.robstride_bus.disconnect(disable_torque=self.config.disable_torque_on_disconnect)
        self.damiao_bus.disconnect(disable_torque=self.config.disable_torque_on_disconnect)

        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
