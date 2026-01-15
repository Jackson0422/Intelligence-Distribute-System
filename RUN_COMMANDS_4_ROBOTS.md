# 4-Robot System Run Commands

## 🎯 Full Startup Flow (4 robots)

### Terminal 1: start Gazebo
```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=4
```

### Terminal 2: start AMCL localization
```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=4
```

### Terminal 3: start collaborative localization
```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=4
```

### Terminal 4: start performance evaluation (optional)
```bash
cd ~/ids_roswk
source install/setup.bash
ros2 run localization_evaluation pose_eval_coloc --ros-args -p num_robots:=4
```

### Terminal 5: start trajectory control
```bash
cd ~/ids_roswk
source install/setup.bash
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=4
```

---

## 📋 Other Configurations

### 2 robots (default)
All commands need no parameters, or explicitly set `num_robots:=2`.

```bash
# Terminals 1-3: launch files
ros2 launch localization_evaluation multibot_gazebo.launch.py
ros2 launch localization_evaluation amcl_multibot.launch.py
ros2 launch localization_evaluation decentralized_coloc.launch.py

# Terminals 4-5: run commands
ros2 run localization_evaluation pose_eval_coloc
ros2 run localization_evaluation track_multibot
```

### 3 robots
All commands use `num_robots:=3`.

```bash
# Terminals 1-3: launch files (use := syntax)
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=3
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=3
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=3

# Terminals 4-5: run commands (use --ros-args -p syntax)
ros2 run localization_evaluation pose_eval_coloc --ros-args -p num_robots:=3
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=3
```

---

## 🔤 Command Syntax Reference

| Command type | Parameter format | Example |
|---------|---------|------|
| `ros2 launch` | `param:=value` | `ros2 launch ... num_robots:=4` |
| `ros2 run` | `--ros-args -p param:=value` | `ros2 run ... --ros-args -p num_robots:=4` |

---

## ⚠️ Important Notes

1. **Parameter consistency**: ensure all commands use the same `num_robots` value.
2. **Startup order**: follow the sequence above (Gazebo → AMCL → collaborative localization → evaluation → trajectory control).
3. **Wait time**: wait a few seconds after each terminal starts so nodes fully initialize before starting the next.
4. **Evaluation**: `pose_eval_coloc` is optional and used to evaluate collaborative localization performance.
5. **Ctrl+C exit**: use Ctrl+C to safely stop each terminal; the evaluation node saves results automatically.

---

## 🤖 Robot Configuration

| Robot ID | Start position | Waypoints |
|---------|---------|--------|
| tb3_1 | (-2.0, 0.0) | (-2,0) → (-2,-2) → (0,-2) → (2,0) |
| tb3_2 | (2.0, 2.0) | (2,2) → (0,2) → (-2,0) → (-2,-2) |
| tb3_3 | (-1.0, -1.5) | (-1,-1.5) → (-2,-0.5) → (-2,0.5) → (0,2) |
| tb3_4 | (2.0, 0.0) | (2,0) → (2,-1) → (1,-1.5) → (0,-2) |

---

Generated on: 2026-01-07
