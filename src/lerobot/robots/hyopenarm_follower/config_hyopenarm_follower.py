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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig

LEFT_DEFAULT_JOINTS_LIMITS: dict[str, tuple[float, float]] = {
    "joint_1": (-75.0, 75.0),
    "joint_2": (-90.0, 9.0),
    "joint_3": (-85.0, 85.0),
    "joint_4": (0.0, 135.0),
    "joint_5": (-85.0, 85.0),
    "joint_6": (-40.0, 40.0),
    "joint_7": (-80.0, 80.0),
    "gripper": (-65.0, 0.0),
}

RIGHT_DEFAULT_JOINTS_LIMITS: dict[str, tuple[float, float]] = {
    "joint_1": (-75.0, 75.0),
    "joint_2": (-9.0, 90.0),
    "joint_3": (-85.0, 85.0),
    "joint_4": (0.0, 135.0),
    "joint_5": (-85.0, 85.0),
    "joint_6": (-40.0, 40.0),
    "joint_7": (-80.0, 80.0),
    "gripper": (-65.0, 0.0),
}


@dataclass
class HyOpenArmFollowerConfigBase:
    """Base configuration for a hybrid follower using Robstride + Damiao motors."""

    port: str
    side: str | None = None
    can_interface: str = "socketcan"
    use_can_fd: bool = False
    can_bitrate: int = 1000000
    can_data_bitrate: int = 5000000

    disable_torque_on_disconnect: bool = True
    max_relative_target: float | dict[str, float] | None = None

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    motor_config: dict[str, tuple[int, int, str, str]] = field(
        default_factory=lambda: {
            "joint_1": (0x01, 0x01, "O6", "robstride"),
            "joint_2": (0x02, 0x02, "O6", "robstride"),
            "joint_3": (0x03, 0x03, "O0", "robstride"),
            "joint_4": (0x04, 0x04, "O0", "robstride"),
            "joint_5": (0x05, 0x15, "dm4310", "damiao"),
            "joint_6": (0x06, 0x16, "dm4310", "damiao"),
            "joint_7": (0x07, 0x17, "dm4310", "damiao"),
            "gripper": (0x08, 0x18, "dm4310", "damiao"),
        }
    )

    position_kp: list[float] = field(
        default_factory=lambda: [40.0, 40.0, 30.0, 30.0, 24.0, 31.0, 25.0, 16.0]
    )
    position_kd: list[float] = field(
        default_factory=lambda: [1.0, 1.0, 0.8, 0.8, 0.2, 0.2, 0.2, 0.2]
    )

    joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {

            "joint_1": (-5.0, 5.0),
            "joint_2": (-5.0, 5.0),
            "joint_3": (-5.0, 5.0),
            "joint_4": (0, 5.0),
            "joint_5": (-5.0, 5.0),
            "joint_6": (-5.0, 5.0),
            "joint_7": (-5.0, 5.0),
            "gripper": (-5.0, 0.0),
        }
    )


@RobotConfig.register_subclass("hyopenarm_follower")
@dataclass
class HyOpenArmFollowerConfig(RobotConfig, HyOpenArmFollowerConfigBase):
    pass
