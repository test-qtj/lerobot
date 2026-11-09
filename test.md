# LeRobot 双臂 OpenArm：上电、采集、训练与部署流程

这份记录对应本机已跑通的配置：LeRobot 位于 `/home/test/lerobot`，Conda 环境为 `/home/test/lerobot/.conda-env`，从臂是 `bi_openarm_follower`，主臂是 `bi_hyopenarm_leader`。

> 本机最近确认的 CAN 分配：`can0` 左从臂、`can1` 右从臂（CAN FD）；`can3` 左主臂、`can2` 右主臂（经典 CAN）。Linux 重启或重新插拔 PCAN 后，接口编号可能改变；每次先检查并验证，不要只凭旧编号连接。

## 0. 上电与安全准备

1. 将两只机械臂和底座固定好，工作区域内清除无关物品，手指远离夹爪、连杆和关节夹点。
2. 按设备标识和 OpenArm 硬件说明给机械臂供电。电机测试和连接命令不能代替电源检查。
3. 将 PCAN-USB Pro FD 和相机接到电脑。第一次部署或遥操时，确保急停或断电方式在手边。
4. 上电后先观察电机状态灯和供电状态。不要在机械臂正在运动时插拔 CAN、电机或相机线。

## 1. 激活环境并检查设备

每次开新终端先运行：

```bash
conda activate /home/test/lerobot/.conda-env
cd /home/test/lerobot

python -c "import torch; print('PyTorch:', torch.__version__); print('CUDA:', torch.cuda.is_available()); print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'unavailable')"
lsusb | grep -i PCAN
lsusb | grep -i RealSense
ip -br link | grep can
```

确认 Python 环境能找到 GPU，`lsusb` 能看到 PCAN 和 RealSense。查看每个 CAN 接口对应的 USB 设备：

```bash
for i in can0 can1 can2 can3; do
  echo "--- $i ---"
  readlink -f "/sys/class/net/$i/device"
done
```

同一个 PCAN 双通道适配器通常对应一对 CAN 接口。接口路径/编号与本机记录不同时，先找出正确接口，再继续下面的命令。

## 2. 配置 CAN 并确认电机通信

本机已确认的配置是从臂 CAN FD、主臂经典 CAN：

```bash
# 左右从臂：CAN FD，标称 1 Mbps、数据 5 Mbps
lerobot-setup-can --mode=setup --interfaces=can0,can1 --use_fd=true

# 左右主臂：经典 CAN，1 Mbps
lerobot-setup-can --mode=setup --interfaces=can2,can3

ip -br link | grep can
```

测试左右从臂电机。每个接口应找到 8 个电机：

```bash
lerobot-setup-can --mode=test --interfaces=can0,can1 --use_fd=true
```

看到 `8/8 motors found` 表示该总线上的 Damiao 从臂电机有响应。`UP` 只表示 Linux 接口已启用，不等于电机已响应。

主臂是 Robstride 与 Damiao 混合电机；不要把从臂的 Damiao 扫描结果当成主臂握手结果。主臂连接/握手会由后面的 `lerobot-calibrate` 或遥操启动过程验证。如果接口 `UP` 但电机无响应，先查供电、CAN 接口对应关系、线缆和协议/波特率；反复校准不会修复 CAN 通信问题。

## 3. 校准（仅首次使用或机械零位变化后）

现有校准文件位于 Hugging Face LeRobot 缓存目录。先检查：

```bash
find ~/.cache/huggingface/lerobot/calibration/robots/openarm_follower -maxdepth 1 -type f -print
find ~/.cache/huggingface/lerobot/calibration/teleoperators/hyopenarm_leader -maxdepth 1 -type f -print
```

本机使用的设备 ID：

- 从臂：`my_bimanual_follower_left.json`、`my_bimanual_follower_right.json`
- 主臂：`my_bimanual_hyleader_left.json`、`my_bimanual_hyleader_right.json`

### 3.1 校准左右从臂

仅在文件缺失、零位发生变化或准备重新校准时执行：

```bash
lerobot-calibrate \
  --robot.type=bi_openarm_follower \
  --robot.left_arm_config.port=can0 \
  --robot.left_arm_config.side=left \
  --robot.left_arm_config.use_can_fd=true \
  --robot.right_arm_config.port=can1 \
  --robot.right_arm_config.side=right \
  --robot.right_arm_config.use_can_fd=true \
  --robot.id=my_bimanual_follower
```

按终端提示操作。LeRobot 的从臂归零提示要求手臂自然下垂、夹爪闭合，再按 Enter。已有校准文件时，按 Enter 使用现有文件；只有确实要重做时才输入 `c` 并回车。

### 3.2 校准左右主臂

仅在文件缺失、零位发生变化或准备重新校准时执行：

```bash
lerobot-calibrate \
  --teleop.type=bi_hyopenarm_leader \
  --teleop.left_arm_config.port=can3 \
  --teleop.left_arm_config.use_can_fd=false \
  --teleop.right_arm_config.port=can2 \
  --teleop.right_arm_config.use_can_fd=false \
  --teleop.id=my_bimanual_hyleader
```

主臂会分别握手并提示设置参考零位。按设备和终端提示摆放主臂后按 Enter。已有校准文件时，按 Enter 使用现有文件；不要因为每次重启就重新校准。

## 4. 检查相机

运行 RealSense 检测并短暂采集图像：

```bash
lerobot-find-cameras realsense \
  --output-dir=/tmp/openarm-camera-check \
  --record-time-s=3
```

本机确认过的相机：

| 用途 | 相机 | 序列号 | LeRobot 观测键 |
|---|---|---|---|
| 左腕 | D405 | `260322272305` | `observation.images.left_wrist` |
| 右腕 | D405 | `352122273643` | `observation.images.right_wrist` |
| 顶部 | D455 | `141322252011` | `observation.images.right_top` |

图像分辨率使用 640×480、30 FPS。部署前尽量让球和目标容器处于示范数据中的相似位置，并出现在顶部画面和任务相关的腕部画面中。三路相机都应正常出图，不能黑屏。

## 5. 先做主从遥操检查

校准并确认总线后，用主臂控制从臂。下面命令还会把相机图像和状态显示到 Rerun：

```bash
lerobot-teleoperate \
  --robot.type=bi_openarm_follower \
  --robot.id=my_bimanual_follower \
  --robot.left_arm_config.port=can0 \
  --robot.left_arm_config.side=left \
  --robot.left_arm_config.use_can_fd=true \
  --robot.left_arm_config.cameras='{wrist: {type: intelrealsense, serial_number_or_name: "260322272305", fps: 30, width: 640, height: 480}}' \
  --robot.right_arm_config.port=can1 \
  --robot.right_arm_config.side=right \
  --robot.right_arm_config.use_can_fd=true \
  --robot.right_arm_config.cameras='{wrist: {type: intelrealsense, serial_number_or_name: "352122273643", fps: 30, width: 640, height: 480}, top: {type: intelrealsense, serial_number_or_name: "141322252011", fps: 30, width: 640, height: 480}}' \
  --teleop.type=bi_hyopenarm_leader \
  --teleop.id=my_bimanual_hyleader \
  --teleop.left_arm_config.port=can3 \
  --teleop.left_arm_config.use_can_fd=false \
  --teleop.right_arm_config.port=can2 \
  --teleop.right_arm_config.use_can_fd=false \
  --display_data=true
```

先小幅度操作，确认左右主臂分别控制对应从臂、方向正常，且三路相机画面正常。遥操结束后在终端按 `Ctrl+C`。

## 6. 用主臂采集示范数据

示范数据是主臂遥操从臂时记录的。以下示例创建一个新的 10 集数据集；如果目录或数据集已存在，请换一个新的名称，或按 LeRobot 的数据集恢复流程处理，不要覆盖已有数据。

```bash
lerobot-record \
  --robot.type=bi_openarm_follower \
  --robot.id=my_bimanual_follower \
  --robot.left_arm_config.port=can0 \
  --robot.left_arm_config.side=left \
  --robot.left_arm_config.use_can_fd=true \
  --robot.left_arm_config.cameras='{wrist: {type: intelrealsense, serial_number_or_name: "260322272305", fps: 30, width: 640, height: 480}}' \
  --robot.right_arm_config.port=can1 \
  --robot.right_arm_config.side=right \
  --robot.right_arm_config.use_can_fd=true \
  --robot.right_arm_config.cameras='{wrist: {type: intelrealsense, serial_number_or_name: "352122273643", fps: 30, width: 640, height: 480}, top: {type: intelrealsense, serial_number_or_name: "141322252011", fps: 30, width: 640, height: 480}}' \
  --teleop.type=bi_hyopenarm_leader \
  --teleop.id=my_bimanual_hyleader \
  --teleop.left_arm_config.port=can3 \
  --teleop.left_arm_config.use_can_fd=false \
  --teleop.right_arm_config.port=can2 \
  --teleop.right_arm_config.use_can_fd=false \
  --dataset.repo_id=test/openarm_ball_pick_30s_v2 \
  --dataset.root=/home/test/lerobot/datasets/openarm_ball_pick_30s_v2 \
  --dataset.single_task="Pick up the small ball and place it in the target container." \
  --dataset.num_episodes=10 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --dataset.fps=30 \
  --dataset.push_to_hub=false \
  --display_data=true
```

采集键盘控制（需要终端能接收键盘事件）：

- `→`：提前结束当前 episode，保存并进入下一条。
- `←`：放弃并重录当前 episode。
- `Esc`：停止后续采集并退出。
- episode 默认也会在 30 秒时结束；每条之间有 10 秒重置时间。

尽量保留成功、动作完整、相机能看清球和容器的示范。失败示范不要混进训练数据。训练集任务文本和部署任务文本要一致。

### 检查数据集是否完整

```bash
python - <<'PY'
import json
from pathlib import Path

root = Path("/home/test/lerobot/datasets/openarm_ball_pick_30s_v2")
info = json.loads((root / "meta/info.json").read_text())
print("episodes:", info["total_episodes"])
print("frames:", info["total_frames"])
print("videos:", *[str(p) for p in sorted((root / "videos").rglob("*.mp4"))], sep="\n")
PY
```

确认 episode 数和帧数大于 0，三路 MP4 已经出现在 `videos/` 下。

## 7. 训练 ACT 策略

下面示例使用上一步的新数据集从头训练 5000 步。若训练的是本机已经存在的 `openarm_ball_pick_30s`，将 `dataset.repo_id` 和 `dataset.root` 换成相应路径。输出目录必须是新目录；已存在时更换名称。

```bash
lerobot-train \
  --dataset.repo_id=test/openarm_ball_pick_30s_v2 \
  --dataset.root=/home/test/lerobot/datasets/openarm_ball_pick_30s_v2 \
  --policy.type=act \
  --policy.device=cuda \
  --policy.use_amp=true \
  --policy.push_to_hub=false \
  --output_dir=outputs/train/openarm_ball_pick_act_retrain \
  --job_name=openarm_ball_pick_act_retrain \
  --steps=5000 \
  --batch_size=4 \
  --num_workers=2 \
  --save_freq=1000 \
  --wandb.enable=false
```

训练输出 checkpoint 通常位于：

```text
/home/test/lerobot/outputs/train/openarm_ball_pick_act_retrain/checkpoints/005000/pretrained_model
```

本机已训练过的 ACT checkpoint 是：

```text
/home/test/lerobot/outputs/train/openarm_ball_pick_act_smoke/checkpoints/005000/pretrained_model
```

## 8. 用训练好的策略部署到从臂

部署不连接主臂；模型读取从臂状态和三路相机，再直接向左右从臂发动作。每次运行都使用新的评估数据集名和空目录。数据集名称最后一段必须以 `eval_` 开头；目录已存在时换 `runN`。

以下示例用已训练好的 `act_smoke` checkpoint 和 `run3` 评估目录：

```bash
lerobot-record \
  --robot.type=bi_openarm_follower \
  --robot.id=my_bimanual_follower \
  --robot.left_arm_config.port=can0 \
  --robot.left_arm_config.side=left \
  --robot.left_arm_config.use_can_fd=true \
  --robot.left_arm_config.cameras='{wrist: {type: intelrealsense, serial_number_or_name: "260322272305", fps: 30, width: 640, height: 480}}' \
  --robot.right_arm_config.port=can1 \
  --robot.right_arm_config.side=right \
  --robot.right_arm_config.use_can_fd=true \
  --robot.right_arm_config.cameras='{wrist: {type: intelrealsense, serial_number_or_name: "352122273643", fps: 30, width: 640, height: 480}, top: {type: intelrealsense, serial_number_or_name: "141322252011", fps: 30, width: 640, height: 480}}' \
  --policy.path=/home/test/lerobot/outputs/train/openarm_ball_pick_act_smoke/checkpoints/005000/pretrained_model \
  --policy.device=cuda \
  --policy.use_amp=true \
  --dataset.repo_id=test/eval_openarm_ball_pick_act_5000_run3 \
  --dataset.root=/home/test/lerobot/datasets/eval_openarm_ball_pick_act_5000_run3 \
  --dataset.single_task="Pick up the small ball and place it in the target container." \
  --dataset.fps=30 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10 \
  --dataset.num_episodes=1 \
  --dataset.push_to_hub=false \
  --display_data=true
```

策略启动后会立即控制机械臂，最长运行 30 秒。确认三路相机有画面，球和容器的位置/机械臂起始姿态尽量与示范数据一致。若动作不安全，按 `Ctrl+C` 中止。30 秒结束后，LeRobot 还要编码并保存视频；等待编码完成并回到 shell 提示符，不要在 SVT 编码信息出现时按 `Ctrl+C`。

部署评估数据与人工示范训练数据分开保存。策略失败的评估轨迹不要直接当作成功示范加入训练集。

## 9. 常见报错和判断

- `Interface is not UP`：先按正确 CAN 模式重新运行 `lerobot-setup-can --mode=setup ...`，再用 `ip -br link` 确认。
- `0/8 motors found`：接口 UP 不等于电机在线；检查机械臂供电、适配器到机械臂的线、CANH/CANL、接口对应关系和 CAN FD/经典 CAN 模式。校准文件不会让离线电机产生响应。
- `policy.repo_id argument missing`：本机训练不上传 Hub 时，训练命令要有 `--policy.push_to_hub=false`。
- `dataset name does not begin with 'eval_'`：部署评估数据集 ID 的最后一段加 `eval_` 前缀，例如 `test/eval_openarm_ball_pick_act_5000_run4`。
- `FileExistsError` 指向评估目录：每次部署都换 repo ID 和 root 目录名；不要覆盖旧评估结果。
- Rerun 黑屏或只有几百字节：先看终端异常。若在连接机器人/相机前报错，Rerun 窗口可能只留下空记录。
- `Svt[info]` 是视频编码日志。等它自然结束；若在此时按 `Ctrl+C`，数据可能只写了 parquet/临时 MP4，而 `meta/info.json` 仍为 `total_episodes: 0`。

## 10. 本机目前已知的限制

当前 ACT 模型用 10 条、共 7203 帧的示范训练到 5000 步。机械臂会动说明策略推理和从臂链路工作，但不保证已学会稳定抓取；如果任务失败，先回看示范和评估视频、检查起始姿态和目标是否在相机视野内，再补充一致且成功的人工示范并重新训练。单纯增加部署时长不会让策略在线学习。
