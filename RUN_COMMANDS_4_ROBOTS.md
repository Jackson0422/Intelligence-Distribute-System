# 4机器人系统运行命令

## 🎯 完整启动流程（4个机器人）

### 终端1: 启动Gazebo
```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=4
```

### 终端2: 启动AMCL定位
```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=4
```

### 终端3: 启动协同定位
```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=4
```

### 终端4: 启动性能评估（可选）
```bash
cd ~/ids_roswk
source install/setup.bash
ros2 run localization_evaluation pose_eval_coloc --ros-args -p num_robots:=4
```

### 终端5: 启动轨迹控制
```bash
cd ~/ids_roswk
source install/setup.bash
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=4
```

---

## 📋 其他配置

### 2个机器人（默认）
所有命令不需要参数，或者显式指定 `num_robots:=2`

```bash
# 终端1-3: launch文件
ros2 launch localization_evaluation multibot_gazebo.launch.py
ros2 launch localization_evaluation amcl_multibot.launch.py
ros2 launch localization_evaluation decentralized_coloc.launch.py

# 终端4-5: run命令
ros2 run localization_evaluation pose_eval_coloc
ros2 run localization_evaluation track_multibot
```

### 3个机器人
所有命令使用 `num_robots:=3`

```bash
# 终端1-3: launch文件（使用 := 语法）
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=3
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=3
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=3

# 终端4-5: run命令（使用 --ros-args -p 语法）
ros2 run localization_evaluation pose_eval_coloc --ros-args -p num_robots:=3
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=3
```

---

## 🔤 命令语法对照表

| 命令类型 | 参数格式 | 示例 |
|---------|---------|------|
| `ros2 launch` | `参数名:=值` | `ros2 launch ... num_robots:=4` |
| `ros2 run` | `--ros-args -p 参数名:=值` | `ros2 run ... --ros-args -p num_robots:=4` |

---

## ⚠️ 重要提示

1. **参数一致性**: 确保所有命令使用相同的 `num_robots` 值
2. **启动顺序**: 建议按照上述顺序启动（Gazebo → AMCL → 协同定位 → 性能评估 → 轨迹控制）
3. **等待时间**: 每个终端启动后等待几秒钟，确保节点完全初始化后再启动下一个
4. **性能评估**: `pose_eval_coloc` 是可选的，用于评估协同定位性能
5. **Ctrl+C退出**: 使用Ctrl+C安全退出每个终端，性能评估节点会自动保存结果

---

## 🤖 机器人配置

| 机器人ID | 起始位置 | 路径点 |
|---------|---------|--------|
| tb3_0 | (-2.0, 0.0) | (-2,0) → (-2,-2) → (0,-2) → (2,0) |
| tb3_1 | (2.0, 2.0) | (2,2) → (0,2) → (-2,0) → (-2,-2) |
| tb3_2 | (-1.0, -1.5) | (-1,-1.5) → (-2,-0.5) → (-2,0.5) → (0,2) |
| tb3_3 | (2.0, 0.0) | (2,0) → (2,-1) → (1,-1.5) → (0,-2) |

---

生成时间: 2026-01-07

