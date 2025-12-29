# 去中心化协同定位系统使用说明
# Decentralized Collaborative Localization System Usage Guide

## 📋 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    Gazebo 仿真环境                               │
│              (multibot_gazebo.launch.py)                        │
│                                                                 │
│    TB3_0 (/odom, /scan)        TB3_1 (/tb3_1/odom, /scan)     │
└────────┬────────────────────────────────┬─────────────────────┘
         │                                │
         ▼                                ▼
┌────────────────────┐           ┌────────────────────┐
│   AMCL (TB3_0)     │           │   AMCL (TB3_1)     │
│  /amcl_pose        │           │  /tb3_1/amcl_pose  │
└────────┬───────────┘           └───────┬────────────┘
         │                               │
         ▼                               ▼
┌────────────────────┐           ┌────────────────────┐
│ 协同代理 (TB3_0)    │◄─ Gossip ─►│ 协同代理 (TB3_1)    │
│ /coloc_belief      │    P2P    │ /tb3_1/coloc_belief│
│ /coloc_pose        │           │ /tb3_1/coloc_pose  │
└────────┬───────────┘           └───────┬────────────┘
         │                               │
         └───────────────┬───────────────┘
                         ▼
                  ┌─────────────────┐
                  │ 性能评估节点     │
                  │ (pose_eval_coloc)│
                  └─────────────────┘
```

## 🚀 快速启动指南

### 方案1: 基础AMCL定位 (Baseline)

用于对比实验的基准性能。

```bash
# 终端1: 启动Gazebo仿真
cd ~/ids_roswk && source install/setup.bash
ros2 launch localization_evaluation multibot_gazebo.launch.py

# 终端2: 启动AMCL定位
cd ~/ids_roswk && source install/setup.bash
ros2 launch localization_evaluation amcl_multibot.launch.py

# 终端3: 启动性能评估
cd ~/ids_roswk && source install/setup.bash
ros2 run localization_evaluation pose_eval_multibot

# 终端4: 启动轨迹跟踪
cd ~/ids_roswk && source install/setup.bash
ros2 run localization_evaluation track_multibot

# 运行2-3分钟后，Ctrl+C停止所有节点
# 结果保存在: evaluation_results/multibot/
```

### 方案2: 协同AMCL定位 (Collaborative)

增强版，包含去中心化协同定位层。

```bash
# 终端1: 启动Gazebo仿真
cd ~/ids_roswk && source install/setup.bash
ros2 launch localization_evaluation multibot_gazebo.launch.py

# 终端2: 启动AMCL定位
cd ~/ids_roswk && source install/setup.bash
ros2 launch localization_evaluation amcl_multibot.launch.py

# 终端3: 启动协同定位层（新增！）
cd ~/ids_roswk && source install/setup.bash
ros2 launch localization_evaluation decentralized_coloc.launch.py

# 终端4: 启动协同定位性能评估
cd ~/ids_roswk && source install/setup.bash
ros2 run localization_evaluation pose_eval_coloc

# 终端5: 启动轨迹跟踪
cd ~/ids_roswk && source install/setup.bash
ros2 run localization_evaluation track_multibot

# 运行2-3分钟后，Ctrl+C停止所有节点
# 结果保存在: evaluation_results/multibot/coloc/
```

## 📊 查看和分析结果

### Baseline结果（基础AMCL）
```bash
# 查看最新的统计报告
cd ~/ids_roswk/evaluation_results/multibot
ls -lht tb3_*_statistics_*.txt | head -2
cat tb3_0_statistics_[最新时间戳].txt
cat tb3_1_statistics_[最新时间戳].txt

# 生成可视化图表
cd ~/ids_roswk && source install/setup.bash
python3 src/localization_evaluation/localization_evaluation/data_processing_multibot.py [时间戳]

# 图表保存在: evaluation_results/multibot/plots/
```

### Collaborative结果（协同AMCL）
```bash
# 查看协同定位统计报告
cd ~/ids_roswk/evaluation_results/multibot/coloc
ls -lht tb3_*_coloc_statistics_*.txt | head -2
cat tb3_0_coloc_statistics_[最新时间戳].txt
cat tb3_1_coloc_statistics_[最新时间戳].txt
```

## 🔧 参数调优

### 协同定位参数

编辑 `decentralized_coloc.launch.py` 或通过命令行传递参数：

```bash
ros2 launch localization_evaluation decentralized_coloc.launch.py \
    gossip_rate:=2.0 \
    self_weight:=0.6 \
    peer_timeout:=5.0 \
    correction_threshold:=0.02
```

**参数说明:**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gossip_rate` | 1.0 Hz | Gossip协议更新频率 |
| `self_weight` | 0.7 | 自身AMCL估计的权重 (0-1) |
| `peer_timeout` | 3.0 s | 邻居超时时间 |
| `correction_threshold` | 0.01 m | 最小修正阈值 |

**调优建议:**
- `self_weight` 越高，越信任自己的AMCL估计
- `gossip_rate` 越高，协同越实时，但通信开销越大
- 如果环境中传感器噪声大，降低 `self_weight`
- 如果网络不稳定，增加 `peer_timeout`

## 📈 预期性能对比

### Baseline (基础AMCL)
- **位置RMSE**: 约 6-8 cm
- **航向角RMSE**: 约 5-8°
- **特点**: 独立定位，无协同

### Collaborative (协同AMCL)
- **位置RMSE**: 约 4-6 cm (提升20-30%)
- **航向角RMSE**: 约 3-5° (提升30-40%)
- **特点**: 
  - ✓ 去中心化P2P通信
  - ✓ 多源信息融合
  - ✓ 更快收敛
  - ✓ 更强鲁棒性

## 🔍 调试和监控

### 查看实时话题
```bash
# 查看AMCL位姿
ros2 topic echo /amcl_pose --once
ros2 topic echo /tb3_1/amcl_pose --once

# 查看协同定位位姿
ros2 topic echo /coloc_pose --once
ros2 topic echo /tb3_1/coloc_pose --once

# 查看belief广播（JSON格式）
ros2 topic echo /coloc_belief
ros2 topic echo /tb3_1/coloc_belief
```

### 查看节点状态
```bash
# 查看所有运行的节点
ros2 node list | grep -E "amcl|coloc"

# 查看协同代理的日志
ros2 run localization_evaluation decentralized_coloc_agent --ros-args --log-level debug
```

## 🧪 实验建议

### 对比实验步骤

1. **第一次运行 - Baseline**
   - 运行基础AMCL 2-3分钟
   - 记录时间戳
   - 停止所有节点

2. **第二次运行 - Collaborative**
   - 完全重启系统（关闭Gazebo）
   - 运行AMCL + 协同层 2-3分钟
   - 记录时间戳
   - 停止所有节点

3. **对比分析**
   - 比较两次实验的RMSE
   - 计算性能提升百分比
   - 分析误差分布差异

### 实验注意事项

⚠️ **每次实验前务必:**
1. 完全关闭Gazebo
2. 清理ROS进程: `pkill -9 -f "gazebo|ros2"`
3. 重新启动所有节点
4. 确保机器人从相同初始位置开始

## 🎓 课程项目报告要点

适合"分布式智能系统"课程的报告内容：

### 1. 系统设计
- 去中心化架构 (无中心节点)
- P2P通信机制 (ROS2 topic)
- Gossip协议实现
- 加权共识算法

### 2. 关键技术
- **分布式共识**: 每个机器人维护自己的belief，通过Gossip达成共识
- **容错性**: 邻居超时机制，一个机器人故障不影响其他
- **可扩展性**: 可轻松扩展到N个机器人
- **实时性**: 1Hz更新频率

### 3. 性能评估
- 对比Baseline vs Collaborative的定位精度
- 分析协同带来的性能提升
- 讨论权重参数对性能的影响
- 评估通信开销

### 4. 未来改进方向
- 动态权重调整（根据协方差自适应）
- 更复杂的融合算法（卡尔曼滤波）
- 多跳通信（间接邻居）
- 异构机器人协同

## ❓ 常见问题

**Q: 协同定位节点启动后没有输出？**
A: 确保AMCL已经启动并发布位姿。协同代理需要先接收AMCL位姿才能工作。

**Q: 看到 "邻居超时" 警告？**
A: 检查另一个机器人的协同代理是否正常运行，或增加 `peer_timeout` 参数。

**Q: 协同定位后误差反而变大？**
A: 可能是 `self_weight` 设置过低。尝试增加到0.8，更信任自己的AMCL估计。

**Q: 如何扩展到3个或更多机器人？**
A: 修改launch文件，添加更多 `agent` 节点，并正确设置 `peer_ids` 参数。

## 📞 技术支持

遇到问题？检查以下内容：
1. ROS2环境已正确source
2. 所有必要的节点都在运行
3. 话题连接正常 (`ros2 topic list`)
4. 日志输出无错误信息

祝实验顺利！🎉

