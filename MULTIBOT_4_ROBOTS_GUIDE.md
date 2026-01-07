# 4机器人系统使用指南

## 📋 修改总结

本次更新将系统从2机器人扩展到支持2-4个机器人，并支持通过参数配置运行数量。

### ✨ v2.1 更新（最新）

- ✅ **统一参数控制**：所有launch文件和节点都支持 `num_robots` 参数
- ✅ **默认值优化**：默认运行2个机器人（完全后向兼容）
- ✅ **条件启动**：只spawn/启动需要的机器人节点
- ✅ **资源优化**：减少不必要的计算负载
- ✅ **性能评估扩展**：`pose_eval_coloc` 现在支持2-4个机器人的评估

### 新增机器人配置

| 机器人 | 命名空间 | 出生点 | 路径点 |
|--------|---------|--------|--------|
| **tb3_0** | `` (空) | (-2.0, -0.5) | 保持不变 |
| **tb3_1** | `tb3_1` | (0.0, 0.5) | 保持不变 |
| **tb3_2** | `tb3_2` | **(-1.0, -1.5)** | (-1.0, -1.5) → (-2.0, -0.5) → (-2.0, 0.5) → (0.0, 2.0) |
| **tb3_3** | `tb3_3` | **(2.0, 0.0)** | (2.0, 0.0) → (2.0, -1.0) → (1.0, -1.5) → (0.0, -2.0) |

---

## 🚀 使用方法

### 🎯 统一参数控制（推荐）

**所有launch文件现在都支持 `num_robots` 参数！**

#### 默认：2个机器人（tb3_0 + tb3_1）
```bash
# 1. 启动Gazebo（默认2个机器人）
ros2 launch localization_evaluation multibot_gazebo.launch.py

# 2. 启动AMCL（默认2个机器人）
ros2 launch localization_evaluation amcl_multibot.launch.py

# 3. 启动协同定位（默认2个机器人）
ros2 launch localization_evaluation decentralized_coloc.launch.py

# 4. 运行轨迹控制（默认2个机器人）
ros2 run localization_evaluation track_multibot
```

#### 可选：3个机器人
```bash
# 1. 启动Gazebo（3个机器人）
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=3

# 2. 启动AMCL（3个机器人）
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=3

# 3. 启动协同定位（3个机器人）
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=3

# 4. 运行轨迹控制（3个机器人）
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=3
```

#### 可选：4个机器人
```bash
# 所有命令都添加 num_robots:=4 参数
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=4
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=4
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=4
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=4

# 可选：运行协同定位性能评估（支持2-4个机器人）
ros2 run localization_evaluation pose_eval_coloc --ros-args -p num_robots:=4
```

### 5️⃣ 运行评估

```bash
ros2 run localization_evaluation pose_eval_coloc
```

---

## 📁 修改文件清单

### Python代码
- ✅ `localization_evaluation/track_multibot.py` - 添加tb3_2和tb3_3配置，支持参数选择（num_robots）
- ✅ `localization_evaluation/pose_eval_coloc.py` - **v2.1新增**：支持2-4个机器人的动态评估（num_robots参数）
- ✅ `localization_evaluation/decentralized_coloc_agent.py` - 添加TF frame处理（兼容tb3_0特殊命名）

### Launch文件
- ✅ `launch/decentralized_coloc.launch.py` - 添加tb3_2和tb3_3协同定位节点，支持条件启动（num_robots）
- ✅ `launch/multibot_gazebo.launch.py` - 添加tb3_2和tb3_3的spawn配置，支持条件启动（num_robots）
- ✅ `launch/amcl_multibot.launch.py` - 添加tb3_2和tb3_3的AMCL节点，支持条件启动（num_robots）

### 模型文件（新增）
- 🆕 `models/tb3_2/model.sdf` - tb3_2的Gazebo模型
- 🆕 `models/tb3_3/model.sdf` - tb3_3的Gazebo模型

### 参数文件（新增）
- 🆕 `param/nav2_params_tb3_2.yaml` - tb3_2的AMCL参数
- 🆕 `param/nav2_params_tb3_3.yaml` - tb3_3的AMCL参数

---

## 🔧 关键特性

### ✅ 完全后向兼容
- 默认行为保持不变（2个机器人）
- 现有tb3_0和tb3_1配置完全未改动
- 通过参数选择运行数量

### ✅ 坐标系处理
- tb3_0: `base_footprint`（无前缀，保持不变）
- tb3_1/tb3_2/tb3_3: `tb3_X/base_footprint`（有前缀）
- TF查询自动处理命名差异

### ✅ 协同定位
- 每个机器人自动识别所有其他机器人为邻居
- 支持2-4个机器人的协同定位网络
- peer_timeout机制自动处理机器人离线

---

## 📊 TF树结构

```
/map (全局共享)
├── /base_footprint (tb3_0)
│   ├── /base_link
│   └── /base_scan
├── /tb3_1/base_footprint (tb3_1)
│   ├── /tb3_1/base_link
│   └── /tb3_1/base_scan
├── /tb3_2/base_footprint (tb3_2) ← 新增
│   ├── /tb3_2/base_link
│   └── /tb3_2/base_scan
└── /tb3_3/base_footprint (tb3_3) ← 新增
    ├── /tb3_3/base_link
    └── /tb3_3/base_scan
```

---

## 🎯 验证系统

### 检查TF树
```bash
ros2 run tf2_tools view_frames
```

### 检查话题
```bash
# 查看所有机器人的AMCL位姿
ros2 topic list | grep amcl_pose

# 查看协同定位输出
ros2 topic list | grep coloc_pose
```

### 检查节点
```bash
# 查看所有运行的节点
ros2 node list
```

---

## 📊 快速参考表

| num_robots 参数 | 启动的机器人 | 说明 | 命令语法 |
|--------------|-----------|-----|---------|
| `2` (默认)   | tb3_0, tb3_1 | 保持与原系统完全一致 | 可省略参数 |
| `3`          | tb3_0, tb3_1, tb3_2 | 添加第3个机器人 | launch用`:=`, run用`-p` |
| `4`          | tb3_0, tb3_1, tb3_2, tb3_3 | 全部4个机器人 | launch用`:=`, run用`-p` |

### 🔤 命令语法对照

| 命令类型 | 参数格式 | 示例 |
|---------|---------|------|
| `ros2 launch` | `num_robots:=N` | `ros2 launch ... num_robots:=4` |
| `ros2 run` | `--ros-args -p num_robots:=N` | `ros2 run ... --ros-args -p num_robots:=4` |

## 🚨 注意事项

1. **参数一致性**：确保所有launch文件使用相同的 `num_robots` 值
2. **时间同步**：所有节点都使用 `use_sim_time: true`
3. **初始位置**：参数文件中的initial_pose已配置为与spawn位置一致
4. **peer_ids**：协同定位节点的peer_ids包含所有其他机器人（超时机制自动处理不存在的peer）
5. **spawn延迟**：机器人spawn有延迟避免冲突（0s, 2s, 4s, 6s）
6. **条件启动**：只有满足 `num_robots` 条件的机器人才会被启动

---

## 📈 性能建议

- **2个机器人**：默认配置，性能最佳
- **3个机器人**：建议配置，平衡性能和协同效果
- **4个机器人**：最大配置，计算负载最高

---

## 🎓 示例场景

### 场景1：测试新机器人单独运行
```bash
# 只启动Gazebo，手动控制tb3_2
ros2 launch localization_evaluation multibot_gazebo.launch.py
ros2 topic pub /tb3_2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

### 场景2：对比2机器人vs4机器人协同定位效果
```bash
# Terminal 1-3: 启动基础系统
# Terminal 4: 2机器人
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=2

# 或 4机器人
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=4
```

---

**修改完成时间**: $(date)
**修改版本**: v2.0 - 4机器人支持

