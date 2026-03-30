#!/bin/bash
# 清理脚本 - 停止所有集群相关进程

echo "=========================================="
echo "Cleaning up Swarm Processes"
echo "=========================================="
echo ""

# 1. 停止 roslaunch 进程
echo "[1/5] Stopping roslaunch processes..."
if pgrep -f "roslaunch.*swarm" > /dev/null; then
    pkill -9 -f "roslaunch.*swarm"
    echo "  ✅ roslaunch stopped"
else
    echo "  ℹ️  No roslaunch processes found"
fi

# 2. 停止 roscore 和 rosmaster
echo "[2/5] Stopping roscore and rosmaster..."
if pgrep -x "roscore" > /dev/null || pgrep -x "rosmaster" > /dev/null; then
    killall -9 roscore rosmaster 2>/dev/null
    echo "  ✅ roscore/rosmaster stopped"
else
    echo "  ℹ️  No roscore/rosmaster found"
fi

# 3. 停止 PX4 SITL
echo "[3/5] Stopping PX4 SITL..."
if pgrep -x "px4" > /dev/null; then
    killall -9 px4 2>/dev/null
    echo "  ✅ PX4 SITL stopped"
else
    echo "  ℹ️  No PX4 processes found"
fi

echo "  Cleaning PX4 lock and socket remnants..."
rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null || true
echo "  ✅ PX4 lock/socket remnants cleaned"

# 4. 停止 Gazebo
echo "[4/5] Stopping Gazebo..."
if pgrep -f "gzserver|gzclient" > /dev/null; then
    pkill -9 -f "gzserver|gzclient" 2>/dev/null
    echo "  ✅ Gazebo stopped"
else
    echo "  ℹ️  No Gazebo processes found"
fi

# 5. 清理残留的 Python 脚本
echo "[5/5] Cleaning up Python scripts..."
pkill -9 -f "swarm_dynamic_commander" 2>/dev/null
pkill -9 -f "swarm_traj_trigger" 2>/dev/null
echo "  ✅ Python scripts cleaned"

echo ""
echo "=========================================="
echo "✅ Cleanup Complete!"
echo "=========================================="
echo ""
echo "All swarm processes have been stopped."
echo "You can now start a fresh simulation."
