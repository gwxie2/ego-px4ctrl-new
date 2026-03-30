#!/bin/bash
# 完整冒烟测试脚本
# 测试单机、双机、3机场景的基本功能

set -e

echo "=========================================="
echo "Swarm Smoke Test"
echo "=========================================="
echo ""

# 创建临时目录
TEMP_DIR=$(mktemp -d)
trap "rm -rf $TEMP_DIR" EXIT

# 1. 单机测试
echo "[1/5] Single UAV test..."
cat > $TEMP_DIR/test_single.md << 'EOF'
- `drone_0`: start=(0.0,0.0,0.1,0.0), goal=(3.0,0.0,1.0,0.0)
EOF

python3 src/clean_uav_core/scripts/swarm_launch_generator.py \
  --config $TEMP_DIR/test_single.md \
  --output src/clean_uav_core/launch/test_single.launch > /dev/null 2>&1

echo "  Launching single UAV..."
timeout 30s roslaunch clean_uav_core test_single.launch gui:=false > /tmp/single_test.log 2>&1 &
TEST_PID=$!
sleep 25
if ps -p $TEST_PID > /dev/null; then
    kill $TEST_PID 2>/dev/null
    echo "  ✅ Single UAV test passed"
else
    echo "  ❌ Single UAV test FAILED"
    echo "  Check /tmp/single_test.log for details"
    exit 1
fi

# 2. 双机测试
echo "[2/5] Dual UAV test..."
cat > $TEMP_DIR/test_dual.md << 'EOF'
- `drone_0`: start=(0.0,2.0,0.1,0.0), goal=(8.0,2.0,1.0,0.0)
- `drone_1`: start=(0.0,-2.0,0.1,0.0), goal=(8.0,-2.0,1.0,0.0)
EOF

python3 src/clean_uav_core/scripts/swarm_launch_generator.py \
  --config $TEMP_DIR/test_dual.md \
  --output src/clean_uav_core/launch/test_dual.launch > /dev/null 2>&1

echo "  Launching dual UAVs..."
timeout 30s roslaunch clean_uav_core test_dual.launch gui:=false > /tmp/dual_test.log 2>&1 &
TEST_PID=$!
sleep 25
if ps -p $TEST_PID > /dev/null; then
    kill $TEST_PID 2>/dev/null
    echo "  ✅ Dual UAV test passed"
else
    echo "  ❌ Dual UAV test FAILED"
    echo "  Check /tmp/dual_test.log for details"
    exit 1
fi

# 3. 3 机默认配置测试
echo "[3/5] 3-UAV default config test..."
timeout 30s roslaunch clean_uav_core swarm_top_level.launch gui:=false > /tmp/triple_test.log 2>&1 &
TEST_PID=$!
sleep 25
if ps -p $TEST_PID > /dev/null; then
    kill $TEST_PID 2>/dev/null
    echo "  ✅ 3-UAV test passed"
else
    echo "  ❌ 3-UAV test FAILED"
    echo "  Check /tmp/triple_test.log for details"
    exit 1
fi

# 4. 命名空间隔离测试
echo "[4/5] Namespace isolation test..."
timeout 30s roslaunch clean_uav_core swarm_top_level.launch gui:=false > /tmp/namespace_test.log 2>&1 &
TEST_PID=$!
sleep 20

# 检查节点数量
NODE_COUNT=$(rosnode list 2>/dev/null | grep -c "drone_")
if [ "$NODE_COUNT" -ge 9 ]; then
    echo "  ✅ Namespace isolation test passed ($NODE_COUNT nodes)"
else
    echo "  ❌ Namespace isolation test FAILED (only $NODE_COUNT nodes)"
    kill $TEST_PID 2>/dev/null
    exit 1
fi

kill $TEST_PID 2>/dev/null

# 5. 端口配置测试
echo "[5/5] Port configuration test..."
PORTS=$(netstat -tulpn 2>/dev/null | grep "1857[0-2]" | wc -l)
if [ "$PORTS" -ge 3 ]; then
    echo "  ✅ Port configuration test passed ($PORTS ports)"
else
    echo "  ❌ Port configuration test FAILED (only $PORTS ports)"
    exit 1
fi

echo ""
echo "=========================================="
echo "✅ All smoke tests passed!"
echo "=========================================="
