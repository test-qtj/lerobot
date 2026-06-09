conda activate lerobot310
cd lerobot
# 开启 CAN 接口
<!-- sudo ip link set can0 down 
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 down 
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

cansend can0 008#FFFFFFFFFFFFFFFC
candump can0 -->

# 设置 CAN 接口
# 把该 CAN 接口的 发送队列长度 设置为 1000
for can_port in can0 can1 can2 can3; do
    sudo ip link set $can_port txqueuelen 1000
done

lerobot-setup-can --mode=setup --interfaces=can0,can1
lerobot-setup-can --mode=setup --interfaces=can2,can3 --use_fd=true

# 测试电机通信
lerobot-setup-can --mode=test --interfaces=can0,can1
lerobot-setup-can --mode=test --interfaces=can2,can3 --use_fd=true

# 主臂标定
<!-- lerobot-calibrate --config_path ~/lerobot/openarm_partial_leader.yaml


lerobot-calibrate \
    --teleop.type=openarm_leader \
    --teleop.port=can3 \
    --teleop.id=my_openarm_leader
    
    
lerobot-calibrate \
    --teleop.type=hyopenarm_leader \
    --teleop.port=can1 \
    --teleop.id=my_hyopenarm_leader -->

lerobot-calibrate \
--teleop.type=bi_hyopenarm_leader \
--teleop.left_arm_config.port=can1 \
--teleop.right_arm_config.port=can0 \
--teleop.id=my_bimanual_hyleader

# 跟随臂标定
<!-- lerobot-calibrate --config_path ~/lerobot/openarm_partial_follower.yaml


lerobot-calibrate \
    --robot.type=openarm_follower \
    --robot.port=can2 \
    --robot.side=right \
    --robot.id=my_openarm_follower
    
    
lerobot-calibrate \
    --robot.type=hyopenarm_follower \
    --robot.port=can0 \
    --robot.side=right \
    --robot.id=my_hyopenarm_follower


lerobot-calibrate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can2 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can3 \
    --robot.right_arm_config.side=right \
    --robot.id=my_bimanual_follower  -->

lerobot-calibrate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can2 \
    --robot.left_arm_config.side=left \
    --robot.left_arm_config.use_can_fd=true \
    --robot.right_arm_config.port=can3 \
    --robot.right_arm_config.side=right \
    --robot.right_arm_config.use_can_fd=true \
    --robot.id=my_bimanual_follower

# 跟随模式
<!-- lerobot-teleoperate --config_path ~/lerobot/openarm_partial_teleop.yaml


lerobot-teleoperate \
    --robot.type=openarm_follower \
    --robot.port=can0 \
    --robot.side=right \
    --robot.id=my_openarm_follower \
    --teleop.type=openarm_leader \
    --teleop.port=can1 \
    --teleop.id=my_openarm_leader


lerobot-teleoperate \   .port=can0 \
    --robot.side=right \
    --robot.id=my_hyopenarm_follower \
    --teleop.type=hyopenarm_leader \
    --teleop.port=can1 \
    --teleop.id=my_hyopenarm_leader -->
    
lerobot-teleoperate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can2 \
    --robot.left_arm_config.use_can_fd=true \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can3 \
    --robot.right_arm_config.use_can_fd=true \
    --robot.right_arm_config.side=right \
    --robot.id=my_bimanual_follower \
    --teleop.type=bi_hyopenarm_leader \
    --teleop.left_arm_config.port=can1 \
    --teleop.right_arm_config.port=can0 \
    --teleop.id=my_bimanual_hyleader

# 遥操作 显示摄像头画面

lerobot-teleoperate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can2 \
    --robot.left_arm_config.use_can_fd=true \
    --robot.left_arm_config.side=left \
    --robot.left_arm_config.cameras='{
        wrist_left: {"type": "intelrealsense", "serial_number_or_name": "260322272305", "width": 640, "height": 480, "fps": 30},
        top: {"type": "intelrealsense", "serial_number_or_name": "141322252011", "width": 640, "height": 480, "fps": 30},
        }' \
    --robot.right_arm_config.port=can3 \
    --robot.right_arm_config.cameras='{
        wrist_right: {"type": "intelrealsense", "serial_number_or_name": "352122273643", "width": 640, "height": 480, "fps": 30},
        }' \
    --robot.right_arm_config.use_can_fd=true \
    --robot.right_arm_config.side=right \
    --robot.id=my_bimanual_follower \
    --teleop.type=bi_hyopenarm_leader \
    --teleop.left_arm_config.port=can1 \
    --teleop.right_arm_config.port=can0 \
    --teleop.id=my_bimanual_hyleader
    --display_data=true

# hugging face
export http_proxy=http://127.0.0.1:7890
export https_proxy=http://127.0.0.1:7890
pip install "huggingface_hub[cli]"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
hf auth login  #输入token
hf auth whoami #查看user
export HF_USER=" "

# 采集数据集
lerobot-record \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can3 \
    --robot.left_arm_config.use_can_fd=true \
    --robot.right_arm_config.port=can2 \
    --robot.right_arm_config.use_can_fd=true \
    --robot.id=my_bimanual_follower \
    --robot.left_arm_config.cameras='{wrist: {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 60}}' \
    --robot.right_arm_config.cameras='{wrist: {"type": "opencv", "index_or_path": 1, "width": 640, "height": 480, "fps": 60}}' \
    --robot.cameras='{top: {"type": "realsense", "width": 640, "height": 480, "fps": 90}}' \
    --dataset.repo_id=${HF_USER}/bi_openarm_test \  #token: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
    --dataset.single_task="  " \ #
    --dataset.num_episodes=50 \
    --dataset.episode_time_s=60 \
    --dataset.streaming_encoding=true \
    --dataset.encoder_threads=2 \
    --display_data=true

lerobot-record \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can2 \
    --robot.left_arm_config.use_can_fd=true \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can3 \
    --robot.right_arm_config.use_can_fd=true \
    --robot.right_arm_config.side=right \
    --robot.id=my_bimanual_follower \
    --robot.left_arm_config.cameras='{
        wrist_left: {"type": "intelrealsense", "serial_number_or_name": "260322272305", "width": 640, "height": 480, "fps": 30},
        top: {"type": "intelrealsense", "serial_number_or_name": "141322252011", "width": 640, "height": 480, "fps": 30},
        }' \
    --robot.right_arm_config.cameras='{
        wrist_right: {"type": "intelrealsense", "serial_number_or_name": "352122273643", "width": 640, "height": 480, "fps": 30},
        }' \
    --teleop.type=bi_hyopenarm_leader \
    --teleop.left_arm_config.port=can1 \
    --teleop.right_arm_config.port=can0 \
    --teleop.id=my_bimanual_hyleader \
    --dataset.repo_id=xfff67/bi_openarm_test \
    --dataset.root=/home/mizu/lerobot/data/bi_openarm_test \
    --dataset.push_to_hub=false \
    --dataset.single_task="Left gripper picks the light blue stapler into the open black box, then right gripper places the blue duck into the box." \
    --dataset.num_episodes=50 \
    --dataset.episode_time_s=60 \
    --dataset.streaming_encoding=false \
    --dataset.vcodec=h264 \
    --dataset.num_image_writer_processes=1 \
    --dataset.num_image_writer_threads_per_camera=4 \
    --resume=false \
    --display_data=false 



top: {"type": "intelrealsense", "serial_number_or_name": "141322252011", "width": 640, "height": 480, "fps": 30},
wrist_left: {"type": "intelrealsense", "serial_number_or_name": "352122273643", "width": 640, "height": 480, "fps": 30},
wrist_left: {"type": "opencv", "index_or_path": "/dev/video0", "width": 640, "height": 480, "fps": 30}, 
# 右臂d405 id：352122273643
# 左壁d405 id：260322272305
 

    