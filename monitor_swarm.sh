#!/bin/bash
# 集群监控脚本 - 实时显示 UAV 状态

echo "=========================================="
echo "Swarm Status Monitor"
echo "=========================================="
echo "Press Ctrl+C to exit"
echo ""

# 检查 roscore 是否运行
if ! rosnode list > /dev/null 2>&1; then
    echo "❌ roscore is not running!"
    echo "Please run 'roscore' in another terminal."
    exit 1
fi

while true; do
    clear
    echo "=========================================="
    echo "Swarm Status Monitor"
    echo "=========================================="
    date +"%Y-%m-%d %H:%M:%S"

    # UAV 数量
    NUM_UAVS=$(rosnode list 2>/dev/null | grep -c "ego_planner" || echo "0")
    echo ""
    echo "UAVs Running: $NUM_UAVS"

    if [ "$NUM_UAVS" -eq 0 ]; then
        echo "⏳ Waiting for UAVs to start..."
        sleep 2
        continue
    fi

    # 里程计频率
    echo ""
    echo "Odometry Rates:"
    for i in $(seq 0 $((NUM_UAVS - 1))); do
        ODOM_TOPIC="/drone_${i}/odom"
        RATE=$(timeout 1s rostopic hz $ODOM_TOPIC 2>/dev/null | grep -oP '\d+\.\d+(?=\s*Hz)' || echo "0.0")
        printf "  drone_%d: %5s Hz\n" $i $RATE
    done

    # MAVROS 状态
    echo ""
    echo "MAVROS Connection:"
    for i in $(seq 0 $((NUM_UAVS - 1))); do
        STATE_FILE=$(mktemp)
        timeout 1s rostopic echo /drone_${i}/mavros/state > $STATE_FILE 2>/dev/null
        if grep -q "connected: true" $STATE_FILE 2>/dev/null; then
            ARMED=$(grep -oP "armed: \K\w+" $STATE_FILE || echo "false")
            MODE=$(grep -oP "mode: \"\K[^\"]+" $STATE_FILE || echo "UNKNOWN")
            printf "  drone_%d: ✅ Connected | Armed: %-5s | Mode: %s\n" $i $ARMED $MODE
        else
            printf "  drone_%d: ❌ Disconnected\n" $i
        fi
        rm -f $STATE_FILE
    done

    # 飞行模式
    echo ""
    echo "Flight Modes:"
    for i in $(seq 0 $((NUM_UAVS - 1))); do
        MODE_FILE=$(mktemp)
        timeout 1s rostopic echo /drone_${i}/px4ctrl/debug > $MODE_FILE 2>/dev/null
        FSM_STATE=$(grep -oP "current_fsm_state: \K\w+" $MODE_FILE || echo "UNKNOWN")
        printf "  drone_%d: %s\n" $i $FSM_STATE
        rm -f $MODE_FILE
    done

    # 目标点
    echo ""
    echo "Current Goals:"
    for i in $(seq 0 $((NUM_UAVS - 1))); do
        GOAL_FILE=$(mktemp)
        timeout 1s rostopic echo /drone_${i}/goal > $GOAL_FILE 2>/dev/null
        if [ -s $GOAL_FILE ]; then
            X=$(grep -oP "position:\s*x:\s*\K[0-9.-]+" $GOAL_FILE || echo "0.0")
            Y=$(grep -oP "position:\s*y:\s*\K[0-9.-]+" $GOAL_FILE || echo "0.0")
            Z=$(grep -oP "position:\s*z:\s*\K[0-9.-]+" $GOAL_FILE || echo "0.0")
            printf "  drone_%d: (%6.2f, %6.2f, %6.2f)\n" $i $X $Y $Z
        else
            printf "  drone_%d: No goal\n" $i
        fi
        rm -f $GOAL_FILE
    done

    echo ""
    echo "=========================================="
    echo "Refreshing in 2 seconds..."
    sleep 2
done
