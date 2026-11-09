# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
# Licensed under the Apache License, Version 2.0.

from dataclasses import dataclass, field
from pathlib import Path

from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("bi_openarm_mujoco")
@dataclass
class BiOpenArmMujocoConfig(RobotConfig):
    """Configuration for the bimanual OpenArm MuJoCo simulator backend."""

    scene_path: Path
    fps: int = 30
    camera_width: int = 640
    camera_height: int = 480
    show_viewer: bool = True
    camera_map: dict[str, str] = field(
        default_factory=lambda: {
            "top": "camera_ceiling",
            "left_wrist_left": "camera_wrist_left",
            "right_wrist_right": "camera_wrist_right",
        }
    )
