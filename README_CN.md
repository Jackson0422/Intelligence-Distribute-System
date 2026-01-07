# 多机器人协同定位系统

[English](README.md) | [Español](README_ES.md)

## 📋 项目概述

本项目实现了一个基于ROS2的**去中心化多机器人协同定位系统**。系统结合了自适应蒙特卡洛定位（AMCL）和基于Gossip协议的共识算法，通过机器人之间的通信来提高定位精度。

### 主要特性

- ✅ **多机器人支持**：支持2-4个TurtleBot3机器人
- ✅ **去中心化架构**：无需中央协调器
- ✅ **协同定位**：机器人共享位姿信息以提高精度
- ✅ **RRT路径规划**：生成无碰撞轨迹
- ✅ **实时评估**：位置和方向误差实时跟踪
- ✅ **数据可视化**：全面的性能分析绘图工具

### 系统架构

```
┌─────────────┐
│   Gazebo    │ ← 仿真环境
└──────┬──────┘
       │
┌──────┴──────────────────────────────────────┐
│              ROS2 网络                       │
├──────────────┬───────────────┬───────────────┤
│   AMCL       │  协同定位      │  路径规划      │
│   节点       │  代理         │  与控制        │
│              │               │               │
└──────────────┴───────────────┴───────────────┘
```

## 🛠️ 系统要求

### 软件依赖

- **操作系统**: Ubuntu 22.04 (Jammy)
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **Gazebo**: 11.x

### Python包

```bash
sudo apt install python3-pip
pip3 install numpy matplotlib pandas
```

### ROS2包

```bash
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-turtlebot3-gazebo \
                 ros-humble-tf-transformations
```

## 📦 安装步骤

### 1. 克隆仓库

```bash
cd ~
git clone <repository-url> ids_roswk
cd ids_roswk
```

### 2. 编译工作空间

```bash
cd ~/ids_roswk
colcon build --symlink-install
source install/setup.bash
```

### 3. 设置环境变量

添加到 `~/.bashrc`:

```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ids_roswk/src/localization_evaluation/models
source ~/ids_roswk/install/setup.bash
```

## 🚀 快速开始

### 运行完整系统（3个机器人）

运行完整的协同定位系统需要**5个终端**。

#### 终端1：启动Gazebo仿真

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=3
```

**等待所有机器人在Gazebo中生成后再继续。**

#### 终端2：启动AMCL定位

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=3
```

**等待出现 "Managed nodes are active" 消息。**

#### 终端3：启动协同定位

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=3
```

#### 终端4：启动评估节点

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 run localization_evaluation pose_eval_coloc --ros-args -p num_robots:=3
```

#### 终端5：启动机器人控制器

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=3
```

**机器人现在将沿着预定义的路径点导航。**

## ⚙️ 配置说明

### 机器人数量

系统支持**2、3或4个机器人**。在所有命令中修改 `num_robots` 参数：

```bash
# 2个机器人
num_robots:=2

# 3个机器人（默认）
num_robots:=3

# 4个机器人
num_robots:=4
```

### 机器人起始位置

在 `multibot_gazebo.launch.py` 中定义：

| 机器人  | 位置 (x, y)    | 方向 (yaw)  |
|--------|---------------|-------------|
| TB3_0  | (-2.0, -0.5)  | 0.0 弧度    |
| TB3_1  | (0.0, 0.5)    | 0.0 弧度    |
| TB3_2  | (-1.0, -1.5)  | 0.0 弧度    |
| TB3_3  | (2.0, 0.0)    | 0.0 弧度    |

### 路径点

在 `track_multibot.py` 中定义（第361-393行）：

```python
ALL_ROBOTS_CONFIG = {
    'tb3_0': {
        'start': (-2.0, -0.5),
        'waypoints': [(-2.0, -0.5), (-1.0, -0.5), (-1.0, 0.5), (0.0, 2.0)]
    },
    'tb3_1': {
        'start': (0.0, 0.5),
        'waypoints': [(0.0, 0.5), (1.0, 0.5), (1.0, -0.5), (0.0, -2.0)]
    },
    'tb3_2': {
        'start': (-1.0, -1.5),
        'waypoints': [(-1.0, -1.5), (-2.0, -0.5), (-2.0, 0.5), (0.0, 2.0)]
    },
    'tb3_3': {
        'start': (2.0, 0.0),
        'waypoints': [(2.0, 0.0), (2.0, -1.0), (1.0, -1.5), (0.0, -2.0)]
    }
}
```

## 📊 数据分析

### 自动数据记录

运行 `pose_eval_coloc` 时，数据会自动保存到：

```
~/ids_roswk/evaluation_results/multibot/coloc/
```

生成的文件：
- `tb3_X_coloc_eval_TIMESTAMP.csv` - 原始评估数据
- `tb3_X_coloc_statistics_TIMESTAMP.txt` - 统计摘要

### 可视化结果

使用数据处理脚本生成图表：

```bash
cd ~/ids_roswk
python3 src/localization_evaluation/localization_evaluation/data_processing_coloc.py \
    --timestamp 20260107_164136 \
    --num-robots 3
```

**生成的图表：**
1. `trajectory_comparison` - 真实轨迹 vs 估计轨迹
2. `position_error_comparison` - 位置误差随时间变化
3. `yaw_error_comparison` - 方向误差随时间变化
4. `statistics_comparison` - RMSE、均值、最大误差的柱状图
5. `error_distribution` - 误差分布直方图
6. `xy_error_scatter` - 误差的空间分布散点图

**输出位置：**
```
~/ids_roswk/evaluation_results/multibot/coloc/plots/
```

## 📁 项目结构

```
ids_roswk/
├── src/localization_evaluation/
│   ├── launch/
│   │   ├── multibot_gazebo.launch.py       # Gazebo仿真
│   │   ├── amcl_multibot.launch.py         # AMCL定位
│   │   └── decentralized_coloc.launch.py   # 协同定位
│   ├── localization_evaluation/
│   │   ├── track_multibot.py               # 机器人控制器
│   │   ├── pose_eval_coloc.py              # 评估节点
│   │   ├── decentralized_coloc_agent.py    # 协同定位代理
│   │   ├── pathplan.py                     # RRT路径规划器
│   │   └── data_processing_coloc.py        # 数据可视化
│   ├── param/
│   │   ├── nav2_params_tb3_0.yaml          # AMCL参数
│   │   ├── nav2_params_tb3_1.yaml
│   │   ├── nav2_params_tb3_2.yaml
│   │   └── nav2_params_tb3_3.yaml
│   ├── models/
│   │   ├── tb3_1/model.sdf                 # 机器人模型
│   │   ├── tb3_2/model.sdf
│   │   └── tb3_3/model.sdf
│   └── maps/
│       └── map.yaml                        # 环境地图
└── evaluation_results/                     # 输出数据
```

## 🔬 技术细节

### 协同定位算法

系统实现了基于共识的去中心化协同定位，使用以下公式：

```
x̂ᶜᵢ(t) = x̂ᵢ(t) + Σⱼ∈Nᵢ Kᵢⱼ(x̂ⱼ(t) − x̂ᵢ(t))
```

其中：
- `x̂ᶜᵢ(t)` - 机器人i的协同位姿估计
- `x̂ᵢ(t)` - 本地AMCL位姿估计
- `Kᵢⱼ` - 卡尔曼增益（基于协方差动态计算）
- `Nᵢ` - 相邻机器人集合

### 通信协议

- **架构**：全连接图（所有机器人相互通信）
- **协议**：基于Gossip的共识
- **话题**：`/tb3_X/coloc_pose` 和 `/tb3_X/coloc_belief`
- **更新频率**：~10 Hz

## 🐛 故障排除

### 问题：机器人不移动

**原因**：终端启动顺序不正确或AMCL未初始化。

**解决方案**：
1. 等待终端2出现 "Managed nodes are active" 消息
2. 确保终端3在终端4之前运行
3. 最后启动终端5

### 问题：定位误差大

**原因**：AMCL粒子滤波器尚未收敛。

**解决方案**：启动终端5后等待10-15秒，等待粒子收敛。

### 问题：机器人与障碍物碰撞

**原因**：路径规划器中的机器人半径参数太小。

**解决方案**：增加 `pathplan.py` 中的 `ROBOT_RADIUS`（第91行）：
```python
ROBOT_RADIUS = 0.20  # 从0.12增加到0.20
```

### 问题："文件未找到"错误

**原因**：包未编译或环境未source。

**解决方案**：
```bash
cd ~/ids_roswk
colcon build --packages-select localization_evaluation
source install/setup.bash
```

---

**最后更新**: 2026年1月

