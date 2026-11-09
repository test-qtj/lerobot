# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
# Licensed under the Apache License, Version 2.0.

import math
from functools import cached_property
from pathlib import Path
from queue import Queue

from lerobot.processor import RobotAction
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_bi_openarm_mujoco_keyboard import BiOpenArmMujocoKeyboardConfig


class BiOpenArmMujocoKeyboard(Teleoperator):
    """Joint-space keyboard teleoperator, independent of physical leader arms."""

    config_class = BiOpenArmMujocoKeyboardConfig
    name = "bi_openarm_mujoco_keyboard"

    # Each pair is (increase key, decrease key).
    KEY_BINDINGS = {
        "left_joint_1": ("q", "a"),
        "left_joint_2": ("w", "s"),
        "left_joint_3": ("e", "d"),
        "left_joint_4": ("r", "f"),
        "left_joint_5": ("t", "g"),
        "left_joint_6": ("y", "h"),
        "left_joint_7": ("u", "j"),
        "left_gripper": ("i", "k"),
        "right_joint_1": ("1", "2"),
        "right_joint_2": ("3", "4"),
        "right_joint_3": ("5", "6"),
        "right_joint_4": ("7", "8"),
        "right_joint_5": ("9", "0"),
        "right_joint_6": ("-", "="),
        "right_joint_7": ("[", "]"),
        "right_gripper": (";", "'"),
    }
    JOINT_NAMES = {
        f"{side}_{joint}": (
            f"openarm_{side}_joint{index}"
            if joint != "gripper"
            else f"openarm_{side}_finger_joint1"
        )
        for side in ("left", "right")
        for index, joint in enumerate(
            ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", "gripper"),
            start=1,
        )
    }

    def __init__(self, config: BiOpenArmMujocoKeyboardConfig):
        super().__init__(config)
        self.config = config
        self.listener = None
        self.event_queue: Queue[tuple[str, bool]] = Queue()
        self.pressed: dict[str, bool] = {}
        self.targets: dict[str, float] = {}
        self.limits: dict[str, tuple[float, float]] = {}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{key}.pos": float for key in self.KEY_BINDINGS}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.listener is not None and self.listener.is_alive()

    @property
    def is_calibrated(self) -> bool:
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate
        try:
            import mujoco
            from pynput import keyboard
        except ImportError as exc:
            raise ImportError(
                "MuJoCo and pynput are required for bi_openarm_mujoco_keyboard. "
                "Install MuJoCo with `python -m pip install '.[openarm-mujoco]'`."
            ) from exc

        scene_path = Path(self.config.scene_path).expanduser().resolve()
        if not scene_path.is_file():
            raise FileNotFoundError(f"MuJoCo scene not found: {scene_path}")
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(model, data, key_id)

        for key, joint_name in self.JOINT_NAMES.items():
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise ValueError(f"Joint '{joint_name}' is absent from scene {scene_path}")
            qpos_index = model.jnt_qposadr[joint_id]
            sign = -1.0 if key == "left_gripper" else 1.0
            self.targets[key] = sign * math.degrees(float(data.qpos[qpos_index]))
            low, high = model.jnt_range[joint_id]
            lo_deg, hi_deg = sign * math.degrees(float(low)), sign * math.degrees(float(high))
            self.limits[key] = (min(lo_deg, hi_deg), max(lo_deg, hi_deg))

        def on_press(key):
            if hasattr(key, "char") and key.char is not None:
                self.event_queue.put((key.char.lower(), True))

        def on_release(key):
            if hasattr(key, "char") and key.char is not None:
                self.event_queue.put((key.char.lower(), False))

        self.listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        self.listener.start()

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        while not self.event_queue.empty():
            key, is_pressed = self.event_queue.get_nowait()
            self.pressed[key] = is_pressed

        for motor, (increase_key, decrease_key) in self.KEY_BINDINGS.items():
            direction = int(self.pressed.get(increase_key, False)) - int(self.pressed.get(decrease_key, False))
            if direction:
                low, high = self.limits[motor]
                self.targets[motor] = min(
                    high,
                    max(low, self.targets[motor] + direction * self.config.step_size_deg),
                )
        return {f"{key}.pos": value for key, value in self.targets.items()}

    def send_feedback(self, feedback: dict) -> None:
        del feedback

    @check_if_not_connected
    def disconnect(self) -> None:
        self.listener.stop()
        self.listener = None
