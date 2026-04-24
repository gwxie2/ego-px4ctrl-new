# 搜索策略开发进度说明

**日期**：2026-04-23  
**范围**：V2 协同搜索、mission manager 绕障、benchmark 接入、启动链路稳定化

## 1. 这份文档要解决什么问题

当前这条搜索链路已经不是“完全没接通”的状态，但它仍然表现出明显的启发式特征：

- 能跑到搜索阶段。
- 能根据 mission manager 下发目标。
- 能把 benchmark 会话、覆盖率、replan 和目标发现信息落盘。
- 但搜索策略本身仍然偏死板，遇到墙体时仍可能直接朝障碍推进，而不是稳定地先绕开再继续搜索。

所以这份文档的目标不是“宣布完成”，而是把当前真实进度分成三类讲清楚：

1. 我已经做了什么。
2. 我还没做什么。
3. 后续应该往哪里改，才能把“会撞墙的启发式搜索”收敛成“可解释、可回放、能主动绕障”的搜索策略。

## 2. 当前阶段的总体判断

### 2.1 已经到达的阶段

系统已经从“只会启动”推进到“能在 benchmark 闭环里持续暴露搜索行为”的阶段。现在至少具备以下能力：

- mission manager 可以接管搜索目标发布。
- benchmark manager 可以自动起 session 并记录会话级指标。
- coverage engine 可以落盘 coverage history、target detection、summary 和统计图。
- launch 链路已经把安全点云、启动 yaw 稳定化参数、V2 runtime 参数传到了相应节点。

### 2.2 目前仍然存在的问题

最核心的问题没有消失：

- 搜索路径仍然过于依赖局部启发式判断。
- 遇到墙体附近的起点时，策略仍可能把“向前推进”当作默认选择。
- 这意味着它还不是一个“遇障后自动强制改道”的硬约束式策略。

换句话说，当前版本已经比最早的“固定点直飞”强，但还没有强到可以放心地说“不会主动碰壁”。

## 3. 我已经做了什么

### 3.1 搜索逻辑已经从固定目标，改成 mission-manager 级调度

我把搜索目标控制权从“启动时的静态 preset waypoint”往 mission manager 上收拢，让搜索阶段的目标由任务层统一调度，而不是只靠最初下发的一次性点位。

对应文件：

- [src/clean_uav_core/scripts/swarm_mission_manager.py](../src/clean_uav_core/scripts/swarm_mission_manager.py)

现在 mission manager 至少承担了这些职责：

- 根据无人机状态决定下一步搜索目标。
- 在搜索阶段维护 waypoint queue，而不是单点式推进。
- 给不同无人机分配搜索扇区，避免所有机体同时挤向同一区域。
- 记录已访问 waypoint 和失败段 memory，避免重复走回头路。

### 3.2 加入了安全点云输入，开始按 gridmap 风险做绕障判断

我把 mission manager 接到了 planner 侧的安全点云，也就是 inflated occupancy 对应的安全证据。这样它不再只是看任务点本身，而是能参考“当前直线段是不是危险”来决定是否要改成折线。

对应文件：

- [src/plan_env_v2/src/grid_map.cpp](../src/plan_env_v2/src/grid_map.cpp)
- [src/clean_uav_core/scripts/swarm_mission_manager.py](../src/clean_uav_core/scripts/swarm_mission_manager.py)

当前已经实现的行为包括：

- 订阅 inflated occupancy 安全点云。
- 为候选路线计算 segment risk。
- 一旦直达路径风险过高，就尝试生成 bend-point 路线。
- 对失败过的 segment 增加 memory penalty。

这一步的意义是：搜索不再完全靠“看起来像能飞过去”，而是开始把 gridmap 结果纳入决策。

### 3.3 增加了折线路由和路由队列

我没有一上来就改低层控制器，而是在 mission manager 层做了更轻量、也更容易调试的折线绕障：

- 目标不再是一个孤立终点。
- 当前目标可以展开成 route queue。
- 一条 route 里可以包含 bend point，再继续回到原搜索方向。

这样做的好处是：

- 即使直线段不安全，也可以先拐一下再恢复搜索。
- 逻辑更容易从日志里回放。
- 不必立刻侵入 px4ctrl 或 ego_planner 的底层控制循环。

### 3.4 引入了初始朝向相关的逃逸偏置

我已经开始把 UAV 初始朝向和第一段搜索路径绑定起来，不再让第一段目标完全独立于机头方向。

这部分的作用是：

- 当无人机起点本来就贴墙时，第一段不会无脑朝“默认搜索 lane”推进。
- 先按 yaw 方向和安全余量生成 escape candidate。
- 再从候选里挑一个更不容易贴墙的方向。

对应位置仍在：

- [src/clean_uav_core/scripts/swarm_mission_manager.py](../src/clean_uav_core/scripts/swarm_mission_manager.py)

### 3.5 启动链路已经接入了更保守的 startup yaw 参数

我把 V2 runtime 的 startup yaw hold/release 参数加回了 launch 链路，并同步到生成器，避免重新生成 launch 时丢失这些启动保护。

对应文件：

- [src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch](../src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch)
- [src/clean_uav_core/launch/swarm_top_level_v2_mission_test.launch](../src/clean_uav_core/launch/swarm_top_level_v2_mission_test.launch)
- [src/clean_uav_core/scripts/swarm_launch_generator_yaml.py](../src/clean_uav_core/scripts/swarm_launch_generator_yaml.py)

这部分的目的不是“解决搜索策略本身”，而是减少起步阶段因为 yaw 瞬态带来的贴墙风险。

### 3.6 benchmark 闭环已经打通

我还把 benchmark 体系接进来了，让这套搜索行为不是只靠肉眼看，而是能持续记录统计数据。

对应文件：

- [src/clean_uav_core/scripts/benchmark_manager.py](../src/clean_uav_core/scripts/benchmark_manager.py)
- [src/clean_uav_core/scripts/benchmark_coverage_engine.py](../src/clean_uav_core/scripts/benchmark_coverage_engine.py)

现在已经能拿到或落盘的内容包括：

- coverage history。
- target detection history。
- session summary。
- route-memory 统计。
- replan 相关统计。

这让后续调参不再是“看某一次飞行像不像成功”，而是能靠指标判断是不是在变好。

### 3.7 launch 生成链路已经同步

我还把生成器同步到了最新参数链，避免手写 launch 改了，但 YAML 生成回去又把改动覆盖掉。

这件事很关键，因为现在这套系统有两条事实来源：

1. 手写 launch。
2. YAML 生成 launch。

如果两条链不同步，调试结果很容易被“下一次生成”抹掉。

### 3.8 配套分析工具也已经补上

我补了几类研究分析工具，用来做 campaign 级别的后续分析：

- [tools/benchmark_research/run_benchmark_campaign.py](../tools/benchmark_research/run_benchmark_campaign.py)
- [tools/benchmark_research/collect_campaign_index.py](../tools/benchmark_research/collect_campaign_index.py)
- [tools/benchmark_research/run_single_profile.sh](../tools/benchmark_research/run_single_profile.sh)
- [tools/risk_statistics_analysis.py](../tools/risk_statistics_analysis.py)
- [tools/pareto_frontier_analysis.py](../tools/pareto_frontier_analysis.py)
- [tools/parameter_sensitivity_analysis.py](../tools/parameter_sensitivity_analysis.py)

这些工具的作用不是直接修避障，而是把后续的 memory / risk / frontier 结果变成能比较、能归档、能做回归的产物。

### 3.9 目前这批代码已经打下了什么基础

从现在的实现看，代码已经把“搜索”从单点目标下发，推进到了一个可以持续演化的闭环框架。这个基础主要体现在四个方面：

- 任务层已经接管了搜索编排，意味着后续改进应该集中在 mission manager 的选路、分配和回退逻辑，而不是回到控制器里重写飞行基础行为。
- gridmap 安全点云、benchmark 记录和 route memory 已经连成链路，意味着后续可以直接围绕“哪条路更安全、更稳定、更少重试”做优化，而不是先补数据通路。
- launch 和 generator 已经同步，意味着后续参数迭代可以在同一套配置语义下进行，重点应放在策略参数、fallback 条件和约束阈值上，而不是手工维护多个版本的 launch。
- 启动 yaw 和初始朝向偏置已经进入链路，意味着系统已经具备“从起步阶段就考虑墙体风险”的入口，后续应把这一入口继续收硬，尤其是首段逃逸和近墙起步。

这些基础的意义是：后续工作不需要从“能不能飞、能不能接通”这种底层问题重新开始，而应该直接进入“怎么更安全地选路、怎么更稳定地绕障、怎么更可靠地证明它真的有效”的阶段。

因此，未来所作应优先集中在这些代码面：

- [src/clean_uav_core/scripts/swarm_mission_manager.py](../src/clean_uav_core/scripts/swarm_mission_manager.py)：把启发式绕障收紧为强制 detour 规则，完善 route queue、memory penalty 和首段安全约束。
- [src/clean_uav_core/scripts/benchmark_coverage_engine.py](../src/clean_uav_core/scripts/benchmark_coverage_engine.py) 和 [src/clean_uav_core/scripts/benchmark_manager.py](../src/clean_uav_core/scripts/benchmark_manager.py)：继续强化可观测性和回归指标，让“是否更安全”能被稳定量化。
- [src/clean_uav_core/launch/swarm_top_level_v2_mission_test.launch](../src/clean_uav_core/launch/swarm_top_level_v2_mission_test.launch)、[src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch](../src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch) 和 [src/clean_uav_core/scripts/swarm_launch_generator_yaml.py](../src/clean_uav_core/scripts/swarm_launch_generator_yaml.py)：继续维护启动参数一致性，把新增约束和保护参数稳定传下去。
- [src/plan_env_v2/src/grid_map.cpp](../src/plan_env_v2/src/grid_map.cpp) 及其邻近规划链路：如果 mission-manager 级别的启发式仍不足，就把更强的局部可行路径搜索继续往下压到更明确的规划层。

## 4. 现在还没做什么

### 4.1 还没有把“绕障”做成硬约束

当前绕障仍然主要是 mission-manager 级别的启发式逻辑，不是 planner/controller 级别的强约束。

这意味着：

- 只要风险判断不够强，系统还是可能选到贴墙路线。
- 只要 safety 证据不完整，direct segment 仍可能被选中。
- 只要局部几何太窄，当前策略还不一定能稳定切成真正可行的折线。

### 4.2 还没有把局部 A* 变成强制 fallback

我已经确认了局部 A* 是一条可用的后备路径，但它还没有变成“只要直线不安全就必须启用”的强制路线。

换句话说，现在更像是：

- 有 bend-point 方案。
- 有 memory penalty。
- 有 risk score。
- 但还没有到“局部窗口里必须求出可行 polyline，否则不下发”的程度。

### 4.3 还没有形成真正的风险共识

现在的 memory 和 risk 主要还是局部/任务层启发式，不是完整的群体一致性机制。

还缺的东西包括：

- 跨无人机共享的 risk map。
- frontier 的全局协商。
- 对窄走廊的分配抑制。
- 对重复热点的群体级回避。

### 4.4 还没有完成“不会主动碰壁”的实测闭环

目前我已经能看到搜索阶段的行为，但还没有拿到足够长时间、足够多场景的稳定实测，去证明：

- 每次都能避开墙。
- 不会在窄区域里反复重试。
- 不会因为局部代价误判而继续朝障碍推进。

所以这件事还不能宣布完成。

### 4.5 还没有把初始朝向绑定成绝对规则

现在初始朝向已经进入路由打分，但还只是偏置，不是绝对约束。

也就是说：

- 它会影响首段选择。
- 但不会强制“必须沿 yaw 前方走”。
- 如果候选路由质量差，仍可能走向不理想方向。

如果后面实测仍然会贴墙，这一层就要继续加硬。

## 5. 为什么现在还会撞墙

这是当前最需要直说的一点：

> 现在的策略本质上还是“启发式选路”，不是“严格避障规划”。

它现在的问题不是完全没考虑障碍，而是考虑得还不够硬。

具体来说，当前链路仍有三个典型短板：

1. 先验搜索目标仍偏“lane 化”，容易把墙边当成可执行边界。
2. 风险判断对安全点云的依赖很强，如果局部证据不够密，就可能低估危险。
3. 初始朝向只是影响第一段，不足以从根上阻断贴墙起步。

所以你现在看到的现象是合理的：

- 系统已经比最初更聪明；
- 但还没聪明到足以自动绕开所有障碍。

## 6. 后续改进方向

### 6.1 方向一：把“遇障才绕”改成“先判定不可直达就强制绕”

下一步最应该做的是把当前的启发式逻辑收硬：

- 只要 direct segment 穿过 inflated occupancy，就不要再退回直达目标。
- 直接强制启用折线或者局部 A* polyline。
- 折线必须满足最小安全距离和角度变化约束。

这样才能真正把“遇到墙时的被动修正”变成“提前避让”。

### 6.2 方向二：把初始朝向升级为首段约束

如果当前偏置还不足以防撞，就应该把它升级成首段约束：

- 起步时优先沿可行朝向逃离墙边。
- 如果 yaw 前方不安全，就允许逆向逃逸或侧向逃逸。
- 第一段不是搜索主线，而是安全脱困段。

这比单纯的“给朝向加权”更硬，也更适合近墙起步场景。

### 6.3 方向三：把 history memory 做成 corridor 级惩罚图

现在的失败段 memory 还比较轻，下一步应该把它扩展成更明确的 corridor penalty：

- 失败过的 segment 直接降权。
- 低频可行 corridor 给奖励。
- dead-end 区域随时间衰减，但在短期内保持更高惩罚。

这样才能把“曾经撞过的墙附近”真正变成后续搜索的黑名单。

### 6.4 方向四：引入真正的 risk consensus

如果最终目标是无先验搜索，就不能只有单机自己的局部判断，还要有群体级风险共识：

- 哪些 frontier 风险高。
- 哪些 corridor 已经被别的机体占用。
- 哪些区域重复碰壁概率高。
- 哪些目标收益高但代价过大。

这一步做完后，三机行为才会从“各飞各的”变成“知道彼此在避什么”。

### 6.5 方向五：把 benchmark 指标变成回归门槛

后续不要再只看“有没有飞起来”，而要把下面这些变成硬指标：

- collision count。
- replan success rate。
- failed segment hit rate。
- repeated corridor rate。
- frontier coverage progress。
- startup wall-proximity events。

如果这些指标没有变好，就不要把策略宣称为完成。

## 7. 我建议的下一阶段实施顺序

### Phase 1：先把近墙首段修硬

优先做的不是整体大重构，而是把近墙起步修稳：

- 初始 escape 段必须优先于直达搜索段。
- 直接与墙冲突的 segment 不能再被当作正常候选。
- 起步速度和 yaw release 保持保守。

### Phase 2：把折线绕障做成强制 fallback

在 direct segment 不可行时，必须启用：

- 2 到 3 个 bend point。
- 或局部 A* 压缩后的 polyline。

这一阶段的目标是“保证先绕开再搜索”。

### Phase 3：把 memory 和 risk 从局部启发式提升为共享策略

这一步再去做：

- 跨机共享风险摘要。
- 重复走廊惩罚。
- frontier 降权。

### Phase 4：做无先验搜索验证

最后再把隐藏目标彻底隔离，只保留 arbiter / benchmark 侧验证，确认 UAV 侧不需要知道真实目标坐标，也能持续绕障搜索。

## 8. 当前可直接复用的关键文件

- [src/clean_uav_core/scripts/swarm_mission_manager.py](../src/clean_uav_core/scripts/swarm_mission_manager.py)
- [src/clean_uav_core/scripts/benchmark_coverage_engine.py](../src/clean_uav_core/scripts/benchmark_coverage_engine.py)
- [src/clean_uav_core/scripts/benchmark_manager.py](../src/clean_uav_core/scripts/benchmark_manager.py)
- [src/clean_uav_core/launch/swarm_top_level_v2_mission_test.launch](../src/clean_uav_core/launch/swarm_top_level_v2_mission_test.launch)
- [src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch](../src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch)
- [src/clean_uav_core/scripts/swarm_launch_generator_yaml.py](../src/clean_uav_core/scripts/swarm_launch_generator_yaml.py)
- [docs/swarm_benchmark_status_analysis_and_tuning_CN.md](swarm_benchmark_status_analysis_and_tuning_CN.md)
- [docs/swarm_benchmark_comprehensive_report_CN.md](swarm_benchmark_comprehensive_report_CN.md)

## 9. 一句话结论

当前这条搜索策略已经接入了 gridmap、benchmark 和记忆层，但它仍然是“启发式折线绕障”，还不是“硬约束式主动避障”。

它已经比最初的固定点直飞进步很多，但你现在看到的“会直接撞墙”说明：下一步必须把首段绕障、局部 polyline fallback 和风险共识继续做硬，否则它仍会在近墙起步场景里表现得比较死板。