# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
# Licensed under the Apache License, Version 2.0.

from dataclasses import dataclass
from pathlib import Path

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("bi_openarm_mujoco_keyboard")
@dataclass
class BiOpenArmMujocoKeyboardConfig(TeleoperatorConfig):
    """Keyboard controls for the bimanual OpenArm MuJoCo scene."""

    scene_path: Path
    step_size_deg: float = 1.0
