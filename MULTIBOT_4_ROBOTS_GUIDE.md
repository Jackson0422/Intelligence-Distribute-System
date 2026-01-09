# 4-Robot System Guide

## 📋 Change Summary

This update extends the system from 2 robots to 2-4 robots, with runtime control via parameters.

### ✨ v2.1 Update (latest)

- ✅ **Unified parameter control**: all launch files and nodes support the `num_robots` parameter
- ✅ **Default optimized**: default runs 2 robots (fully backward compatible)
- ✅ **Conditional startup**: only spawn/start the requested robot nodes
- ✅ **Resource optimization**: reduces unnecessary compute load
- ✅ **Evaluation extended**: `pose_eval_coloc` now supports evaluation for 2-4 robots

### New robot configuration

| Robot | Namespace | Spawn position | Waypoints |
|--------|---------|--------|--------|
| **tb3_0** | `` (empty) | (-2.0, -0.5) | unchanged |
| **tb3_1** | `tb3_1` | (0.0, 0.5) | unchanged |
| **tb3_2** | `tb3_2` | **(-1.0, -1.5)** | (-1.0, -1.5) → (-2.0, -0.5) → (-2.0, 0.5) → (0.0, 2.0) |
| **tb3_3** | `tb3_3` | **(2.0, 0.0)** | (2.0, 0.0) → (2.0, -1.0) → (1.0, -1.5) → (0.0, -2.0) |

---

## 🚀 How to Use

### 🎯 Unified parameter control (recommended)

**All launch files now support the `num_robots` parameter!**

#### Default: 2 robots (tb3_0 + tb3_1)

```bash
# 1. Start Gazebo (default 2 robots)
ros2 launch localization_evaluation multibot_gazebo.launch.py

# 2. Start AMCL (default 2 robots)
ros2 launch localization_evaluation amcl_multibot.launch.py

# 3. Start collaborative localization (default 2 robots)
ros2 launch localization_evaluation decentralized_coloc.launch.py

# 4. Run trajectory control (default 2 robots)
ros2 run localization_evaluation track_multibot
```

#### Optional: 3 robots

```bash
# 1. Start Gazebo (3 robots)
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=3

# 2. Start AMCL (3 robots)
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=3

# 3. Start collaborative localization (3 robots)
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=3

# 4. Run trajectory control (3 robots)
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=3
```

#### Optional: 4 robots

```bash
# Add num_robots:=4 to all commands
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=4
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=4
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=4
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=4

# Optional: run collaborative localization evaluation (supports 2-4 robots)
ros2 run localization_evaluation pose_eval_coloc --ros-args -p num_robots:=4
```

### 5️⃣ Run evaluation

```bash
ros2 run localization_evaluation pose_eval_coloc
```

---

## 📁 Modified File List

### Python code

- ✅ `localization_evaluation/track_multibot.py` - add tb3_2 and tb3_3 configs, support parameter selection (`num_robots`)
- ✅ `localization_evaluation/pose_eval_coloc.py` - **v2.1 new**: dynamic evaluation for 2-4 robots (`num_robots` parameter)
- ✅ `localization_evaluation/decentralized_coloc_agent.py` - add TF frame handling (compatible with tb3_0 naming)

### Launch files
- ✅ `launch/decentralized_coloc.launch.py` - add tb3_2 and tb3_3 collaborative nodes, conditional startup (`num_robots`)
- ✅ `launch/multibot_gazebo.launch.py` - add tb3_2 and tb3_3 spawn configs, conditional startup (`num_robots`)
- ✅ `launch/amcl_multibot.launch.py` - add tb3_2 and tb3_3 AMCL nodes, conditional startup (`num_robots`)

### Model files (new)
- 🆕 `models/tb3_2/model.sdf` - Gazebo model for tb3_2
- 🆕 `models/tb3_3/model.sdf` - Gazebo model for tb3_3

### Parameter files (new)
- 🆕 `param/nav2_params_tb3_2.yaml` - AMCL params for tb3_2
- 🆕 `param/nav2_params_tb3_3.yaml` - AMCL params for tb3_3

---

## 🔧 Key Features

### ✅ Fully backward compatible
- Default behavior unchanged (2 robots)
- Existing tb3_0 and tb3_1 configs are untouched
- Select the run size via parameters

### ✅ Coordinate frames
- tb3_0: `base_footprint` (no prefix, unchanged)
- tb3_1/tb3_2/tb3_3: `tb3_X/base_footprint` (prefixed)
- TF lookup automatically handles naming differences

### ✅ Collaborative localization
- Each robot automatically discovers all other robots as neighbors
- Supports 2-4 robot collaborative networks
- `peer_timeout` handles offline robots automatically

---

## 📊 TF Tree Structure

```
/map (global shared)
├── /base_footprint (tb3_0)
│   ├── /base_link
│   └── /base_scan
├── /tb3_1/base_footprint (tb3_1)
│   ├── /tb3_1/base_link
│   └── /tb3_1/base_scan
├── /tb3_2/base_footprint (tb3_2) <- new
│   ├── /tb3_2/base_link
│   └── /tb3_2/base_scan
└── /tb3_3/base_footprint (tb3_3) <- new
    ├── /tb3_3/base_link
    └── /tb3_3/base_scan
```

---

## 🎯 System Validation

### Check TF tree
```bash
ros2 run tf2_tools view_frames
```

### Check topics
```bash
# View AMCL poses for all robots
ros2 topic list | grep amcl_pose

# View collaborative localization output
ros2 topic list | grep coloc_pose
```

### Check nodes
```bash
# List all running nodes
ros2 node list
```

---

## 📊 Quick Reference

| num_robots parameter | Robots started | Description | Command syntax |
|--------------|-----------|-----|---------|
| `2` (default)   | tb3_0, tb3_1 | Fully matches the original system | param optional |
| `3`          | tb3_0, tb3_1, tb3_2 | Adds the 3rd robot | launch uses `:=`, run uses `-p` |
| `4`          | tb3_0, tb3_1, tb3_2, tb3_3 | All 4 robots | launch uses `:=`, run uses `-p` |

### 🔤 Command syntax reference

| Command type | Parameter format | Example |
|---------|---------|------|
| `ros2 launch` | `num_robots:=N` | `ros2 launch ... num_robots:=4` |
| `ros2 run` | `--ros-args -p num_robots:=N` | `ros2 run ... --ros-args -p num_robots:=4` |

## 🚨 Notes

1. **Parameter consistency**: ensure all launch files use the same `num_robots` value.
2. **Time sync**: all nodes use `use_sim_time: true`.
3. **Initial poses**: `initial_pose` in parameter files matches the spawn positions.
4. **peer_ids**: collaborative localization nodes include all other robots; timeouts handle missing peers.
5. **Spawn delays**: robot spawns are delayed to avoid conflicts (0s, 2s, 4s, 6s).
6. **Conditional startup**: only robots that satisfy `num_robots` are launched.

---

## 📈 Performance Suggestions

- **2 robots**: default configuration, best performance
- **3 robots**: recommended configuration, balances performance and collaboration
- **4 robots**: maximum configuration, highest compute load

---

## 🎓 Example Scenarios

### Scenario 1: test a new robot running alone
```bash
# Start Gazebo only, manually control tb3_2
ros2 launch localization_evaluation multibot_gazebo.launch.py
ros2 topic pub /tb3_2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```

### Scenario 2: compare 2-robot vs 4-robot collaborative localization
```bash
# Terminal 1-3: start the base system
# Terminal 4: 2 robots
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=2

# Or 4 robots
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=4
```

---
