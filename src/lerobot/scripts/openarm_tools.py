from dataclasses import dataclass, field
import draccus
from pprint import pprint
import logging
from lerobot.robots import (  # noqa: F401
    make_robot_from_config,
    openarm_follower,
    RobotConfig,
)
from lerobot.teleoperators import (  # noqa: F401
    make_teleoperator_from_config,
    openarm_leader,
    TeleoperatorConfig,
)
from lerobot.utils.import_utils import register_third_party_plugins
import time

@dataclass
class OpenarmtoolConfig:
    teleop: TeleoperatorConfig = field(
        default_factory=lambda: openarm_leader.OpenArmLeaderConfig(
            port="can1",
            id="my_openarm_leader",
        )
    )
    robot: RobotConfig = field(
        default_factory=lambda: openarm_follower.OpenArmFollowerConfig(
            port="can0",
        )
    )

    display_data: bool = False
    # Display data on a remote Rerun server
    display_ip: str | None = None
    # Port of the remote Rerun server
    display_port: int | None = None
    # Whether to display compressed images in Rerun
    display_compressed_images: bool = False

    # 每隔多少秒读取一次
    interval_s: float = 10.0


def extract_joint_pos(data: dict) -> dict:
    """只提取 .pos 结尾的关节角度字段"""
    return {k: v for k, v in data.items() if k.endswith(".pos")}


@draccus.wrap()
def main(cfg: OpenarmtoolConfig):
    register_third_party_plugins()

    teleop = make_teleoperator_from_config(cfg.teleop)
    robot = make_robot_from_config(cfg.robot)

    teleop.connect(calibrate=False)
    robot.connect(calibrate=False)

    try:
        print(f"开始读取，每隔 {cfg.interval_s} 秒读取一次，按 Ctrl+C 退出。")

        while True:
            # 读取 leader 当前状态
            leader_action = teleop.get_action()
            leader_joint_angles = extract_joint_pos(leader_action)

            # 读取 follower 当前状态
            follower_obs = robot.get_observation()
            follower_joint_angles = extract_joint_pos(follower_obs)

            print("\n========== Leader 当前关节角度 ==========")
            pprint(leader_joint_angles)

            print("\n========== Follower 当前关节角度 ==========")
            pprint(follower_joint_angles)

            print("\n" + "=" * 60)
            time.sleep(cfg.interval_s)

    except KeyboardInterrupt:
        print("\n用户中断, 程序退出。")

    finally:
        teleop.disconnect()
        robot.disconnect()


if __name__ == "__main__":
    main()