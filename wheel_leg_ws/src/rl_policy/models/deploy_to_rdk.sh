#!/bin/bash
# Deploy RL policy model from PC to RDK X5
#
# Prerequisites:
#   1. Trained model at models/rl_stand.zip
#   2. Exported ONNX model (see train_stand.py --export)
#   3. RDK X5 SSH access (<RDK_IP>)

set -e

RDK_IP="<RDK_IP>"
RDK_USER="sunrise"
RDK_MODEL_DIR="/home/sunrise/models"

echo "=== Wheel-Legged RL Policy Deployment ==="
echo ""

# 1. Copy model to RDK
echo "[1/4] Uploading ONNX model..."
scp models/rl_stand.onnx ${RDK_USER}@${RDK_IP}:${RDK_MODEL_DIR}/

# 2. Copy launch file
echo "[2/4] Uploading launch file..."
scp ../src/rl_policy/launch/rl_policy.launch.py ${RDK_USER}@${RDK_IP}:${RDK_MODEL_DIR}/

# 3. Build the ROS2 workspace on RDK
echo "[3/4] Building ROS2 workspace on RDK..."
ssh ${RDK_USER}@${RDK_IP} << 'EOF'
source /opt/tros/humble/setup.bash
cd ~/wheel_leg_ws
colcon build --symlink-install --packages-select rl_policy
source install/setup.bash
echo "Build complete!"
EOF

# 4. Instructions
echo ""
echo "[4/4] Deployment complete!"
echo ""
echo "To run on RDK X5:"
echo "  ssh ${RDK_USER}@${RDK_IP}"
echo "  source /opt/tros/humble/setup.bash"
echo "  source ~/wheel_leg_ws/install/setup.bash"
echo ""
echo "  # Start RL policy (default: velocity mode)"
echo "  ros2 launch rl_policy rl_policy.launch.py"
echo ""
echo "  # Enable RL control (sets rl_mode=1 on STM32)"
echo "  ros2 topic pub /rl_enable std_msgs/msg/Bool '{data: true}' -1"
echo ""
echo "  # Emergency stop"
echo "  ros2 topic pub /rl_enable std_msgs/msg/Bool '{data: false}' -1"
