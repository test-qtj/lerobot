# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
# Licensed under the Apache License, Version 2.0.

import math
from functools import cached_property
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_bi_openarm_mujoco import BiOpenArmMujocoConfig


class BiOpenArmMujoco(Robot):
    """LeRobot Robot adapter for the OpenArm bimanual MuJoCo cell scene.

    The public state/action keys and angular units match BiOpenArmFollower.
    MuJoCo controls and joint positions are converted between radians and degrees.
    """

    config_class = BiOpenArmMujocoConfig
    name = "bi_openarm_mujoco"

    ARM_JOINTS = tuple(f"joint_{i}" for i in range(1, 8)) + ("gripper",)
    DEFAULT_CAMERAS = {
        "top": "camera_ceiling",
        "left_wrist_left": "camera_wrist_left",
        "right_wrist_right": "camera_wrist_right",
    }

    def __init__(self, config: BiOpenArmMujocoConfig):
        super().__init__(config)
        self.config = config
        self.model = None
        self.data = None
        self.renderer = None
        self.viewer = None
        self._connected = False
        # LeRobot's recorder uses this mapping to size its image writer pool.
        self.cameras = dict(config.camera_map)
        self._joint_ids: dict[str, int] = {}
        self._actuator_ids: dict[str, int] = {}
        self._camera_ids: dict[str, int] = {}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        motors = {f"{side}_{joint}.pos": float for side in ("left", "right") for joint in self.ARM_JOINTS}
        images = {
            key: (self.config.camera_height, self.config.camera_width, 3) for key in self.config.camera_map
        }
        return {**motors, **images}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {
            f"{side}_{joint}.pos": float for side in ("left", "right") for joint in self.ARM_JOINTS
        }

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        """Simulation joints do not require hardware calibration."""

    def configure(self) -> None:
        """Simulation actuators are configured by the loaded MJCF scene."""

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        del calibrate  # Calibration is not applicable to a simulator.
        try:
            import mujoco
        except ImportError as exc:
            raise ImportError(
                "MuJoCo is required for bi_openarm_mujoco. Install with "
                "`python -m pip install '.[openarm-mujoco]'`."
            ) from exc

        scene_path = Path(self.config.scene_path).expanduser().resolve()
        if not scene_path.is_file():
            raise FileNotFoundError(f"MuJoCo scene not found: {scene_path}")

        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        mujoco.mj_forward(self.model, self.data)

        for side in ("left", "right"):
            for index, joint in enumerate(self.ARM_JOINTS, start=1):
                model_joint = f"openarm_{side}_joint{index}" if joint != "gripper" else f"openarm_{side}_finger_joint1"
                actuator = f"{side}_joint{index}_ctrl" if joint != "gripper" else f"{side}_finger1_ctrl"
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, model_joint)
                actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
                if joint_id < 0 or actuator_id < 0:
                    raise ValueError(f"Scene is missing OpenArm {side} joint/actuator: {model_joint}, {actuator}")
                key = f"{side}_{joint}"
                self._joint_ids[key] = joint_id
                self._actuator_ids[key] = actuator_id
                qpos_index = self.model.jnt_qposadr[joint_id]
                self.data.ctrl[actuator_id] = self.data.qpos[qpos_index]

        self._camera_ids = {}
        for feature_name, camera_name in self.config.camera_map.items():
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            if camera_id < 0:
                raise ValueError(f"Camera '{camera_name}' for feature '{feature_name}' is absent from the scene")
            self._camera_ids[feature_name] = camera_id

        if self._camera_ids:
            self.renderer = mujoco.Renderer(
                self.model, height=self.config.camera_height, width=self.config.camera_width
            )
        if self.config.show_viewer:
            import mujoco.viewer

            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self._connected = True

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        import mujoco

        steps = max(1, round((1.0 / self.config.fps) / self.model.opt.timestep))
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()

        observation: dict[str, Any] = {}
        for key, joint_id in self._joint_ids.items():
            qpos_index = self.model.jnt_qposadr[joint_id]
            sign = -1.0 if key == "left_gripper" else 1.0
            observation[f"{key}.pos"] = sign * math.degrees(float(self.data.qpos[qpos_index]))

        for feature_name, camera_id in self._camera_ids.items():
            self.renderer.update_scene(self.data, camera=camera_id)
            observation[feature_name] = self.renderer.render().copy()
        return observation

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        sent_action: RobotAction = {}
        for key, value in action.items():
            if not key.endswith(".pos") or value is None:
                continue
            motor_key = key.removesuffix(".pos")
            actuator_id = self._actuator_ids.get(motor_key)
            if actuator_id is None:
                continue
            # LeRobot's existing bimanual CLI flips joints 1-4 before dispatching
            # to a robot. Undo that hardware-side convention here because MJCF
            # already defines the simulated joint axes. The left gripper's MJCF
            # axis is also reversed relative to the follower's degree convention.
            cli_sign = -1.0 if any(motor_key.endswith(f"joint_{i}") for i in range(1, 5)) else 1.0
            gripper_sign = -1.0 if motor_key == "left_gripper" else 1.0
            sign = cli_sign * gripper_sign
            target_rad = math.radians(sign * float(value))
            low, high = self.model.actuator_ctrlrange[actuator_id]
            target_rad = float(np.clip(target_rad, low, high))
            self.data.ctrl[actuator_id] = target_rad
            sent_action[key] = sign * math.degrees(target_rad)
        return sent_action

    @check_if_not_connected
    def disconnect(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        self.model = None
        self.data = None
        self._connected = False
