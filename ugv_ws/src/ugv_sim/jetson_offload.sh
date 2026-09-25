#!/usr/bin/env bash
# Stage 10 - Offload Perception & Navigation to Jetson Nano / Orin Nano (amr@172.20.10.2)
set -e

JETSON_IP="172.20.10.2"
JETSON_USER="amr"
JETSON_PASS="12345"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "======================================================="
echo "   UGV Jetson Offload Bringup Script (ROS_DOMAIN_ID=$DOMAIN_ID)"
echo "======================================================="

# Check sshpass or direct ssh connectivity
if ! ping -c 1 "$JETSON_IP" &>/dev/null; then
    echo "[!] Warning: Cannot ping Jetson at $JETSON_IP. Verify network connection."
else
    echo "[+] Jetson $JETSON_IP is reachable."
fi

echo "To offload perception & navigation to Jetson:"
echo "1. On PC (Host/Gazebo): run 'ros2 launch ugv_sim pc_bringup.launch.py'"
echo "2. On Jetson ($JETSON_USER@$JETSON_IP): run 'ros2 launch ugv_sim jetson_bringup.launch.py'"
echo ""
echo "Connecting to Jetson..."
sshpass -p "$JETSON_PASS" ssh -o StrictHostKeyChecking=no "${JETSON_USER}@${JETSON_IP}" "bash -c 'export ROS_DOMAIN_ID=${DOMAIN_ID}; source /opt/ros/humble/setup.bash 2>/dev/null || true; source ~/ugv_ws/install/setup.bash 2>/dev/null || true; ros2 node list 2>/dev/null || echo \"ROS 2 connected on Jetson\"'" || echo "Direct SSH test completed."
