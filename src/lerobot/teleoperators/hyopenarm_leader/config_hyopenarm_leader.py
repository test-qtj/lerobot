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

from ..config import TeleoperatorConfig


@dataclass
class HyOpenArmLeaderConfigBase:
    """Base configuration for the OpenArms leader/teleoperator with Damiao/Robstride motors."""
    # CAN ports for the two motor families
    # CAN interfaces - one per arm
    # Arm CAN interface (e.g., "can3")
    # Linux: "can0", "can1", etc.
    port: str
    # CAN interface type: "socketcan" (Linux), "slcan" (serial), or "auto" (auto-detect)
    can_interface: str = "socketcan"

    # CAN FD settings (OpenArms uses CAN FD by default)
    use_can_fd: bool = False
    can_bitrate: int = 1000000
    can_data_bitrate: int = 5000000

    # Motor configuration for OpenArms (7 DOF per arm)
    # Maps motor names to (send_can_id, recv_can_id, motor_type)
    # Based on: https://docs.openarm.dev/software/setup/configure-test
    # This configuration uses Damiao motors with 24V motor parameters where
    # the motor family has separate 24V and 48V model entries.
    motor_config: dict[str, tuple[int, int, str, str]] = field(
        default_factory=lambda: {
            "joint_1": (0x01, 0x11, "dm8009", "damiao"),
            "joint_2": (0x02, 0x12, "dm4340", "damiao"),
            "joint_3": (0x03, 0x13, "dm4340", "damiao"),
            "joint_4": (0x04, 0x14, "dm4340", "damiao"),
            "joint_5": (0x05, 0x15, "dm4310", "damiao"),
            "joint_6": (0x06, 0x16, "dm4310", "damiao"),
            "joint_7": (0x07, 0x17, "dm4310", "damiao"),
            "gripper": (0x08, 0x18, "dm4310", "damiao"),
        }
    )

    # Torque mode settings for manual control
    # When enabled, motors have torque disabled for manual movement
    manual_control: bool = True

    # Joint gains, aligned with joint_1 ... gripper
    position_kp: list[float] = field(
        default_factory=lambda: [50.0, 50.0, 50.0, 50.0, 25.0, 8.0, 8.0, 8.0]
    )
    position_kd: list[float] = field(
        default_factory=lambda: [1.0, 1.0, 1.0, 1.0, 0.5, 0.2, 0.2, 0.2]
    )


@TeleoperatorConfig.register_subclass("hyopenarm_leader")
@dataclass
class HyOpenArmLeaderConfig(TeleoperatorConfig, HyOpenArmLeaderConfigBase):
    pass

