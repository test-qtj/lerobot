conda activate lerobot310
cd lerobot
# 开启 CAN 接口
sudo ip link set can0 down 
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 down 
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

cansend can0 008#FFFFFFFFFFFFFFFC
candump can0

# 设置 CAN 接口
lerobot-setup-can --mode=setup --interfaces=can0,can1
lerobot-setup-can --mode=setup --interfaces=can2,can3 --use_fd=true

# 测试电机通信
lerobot-setup-can --mode=test --interfaces=can0,can1
lerobot-setup-can --mode=test --interfaces=can2,can3 --use_fd=true

# 主臂标定
lerobot-calibrate --config_path ~/lerobot/openarm_partial_leader.yaml


lerobot-calibrate \
    --teleop.type=openarm_leader \
    --teleop.port=can3 \
    --teleop.id=my_openarm_leader
    
    
lerobot-calibrate \
    --teleop.type=hyopenarm_leader \
    --teleop.port=can1 \
    --teleop.id=my_hyopenarm_leader

lerobot-calibrate \
--teleop.type=bi_hyopenarm_leader \
--teleop.left_arm_config.port=can0 \
--teleop.right_arm_config.port=can1 \
--teleop.id=my_bimanual_hyleader

# 跟随臂标定
lerobot-calibrate --config_path ~/lerobot/openarm_partial_follower.yaml


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
    --robot.id=my_bimanual_follower 

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
lerobot-teleoperate --config_path ~/lerobot/openarm_partial_teleop.yaml


lerobot-teleoperate \
    --robot.type=openarm_follower \
    --robot.port=can0 \
    --robot.side=right \
    --robot.id=my_openarm_follower \
    --teleop.type=openarm_leader \
    --teleop.port=can1 \
    --teleop.id=my_openarm_leader


lerobot-teleoperate \
    --robot.type=hyopenarm_follower \
    --robot.port=can0 \
    --robot.side=right \
    --robot.id=my_hyopenarm_follower \
    --teleop.type=hyopenarm_leader \
    --teleop.port=can1 \
    --teleop.id=my_hyopenarm_leader
    
lerobot-teleoperate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can3 \
    --robot.left_arm_config.use_can_fd=true \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can2 \
    --robot.right_arm_config.use_can_fd=true \
    --robot.right_arm_config.side=right \
    --robot.id=my_bimanual_follower \
    --teleop.type=bi_hyopenarm_leader \
    --teleop.left_arm_config.port=can1 \
    --teleop.right_arm_config.port=can0 \
    --teleop.id=my_bimanual_hyleader
