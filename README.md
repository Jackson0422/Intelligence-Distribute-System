# Multi-Robot Collaborative Localization System

[Requirements](Requirements.md) | [Quick start](QuickStart.md)

## 📋 Overview

This project implements a **decentralized collaborative localization system** for multiple TurtleBot3 robots in ROS2. The system uses Adaptive Monte Carlo Localization (AMCL) combined with a gossip-based consensus protocol to improve localization accuracy through inter-robot communication.

### Key Features

- **Multi-robot support**: 2-4 TurtleBot3 robots
- **Decentralized architecture**: No central coordinator required
- **Collaborative localization**: Robots share pose information to improve accuracy
- **RRT path planning**: Collision-free trajectory generation
- **Real-time evaluation**: Position and orientation error tracking
- **Data visualization**: Comprehensive plotting tools for performance analysis

### System Architecture

```
┌─────────────┐
│   Gazebo    │ ← Simulation Environment
└──────┬──────┘
       │
┌──────┴──────────────────────────────────────┐
│              ROS2 Network                    │
├──────────────┬───────────────┬───────────────┤
│   AMCL       │  Collaborative│  Path         │
│   Nodes      │  Localization │  Planning     │
│              │  Agents       │  & Control    │
└──────────────┴───────────────┴───────────────┘
```

## Configuration

### Number of Robots

The system supports **2, 3, or 4 robots**. Change the `num_robots` parameter in all commands:

```bash
# For 2 robots
num_robots:=2

# For 3 robots (default)
num_robots:=3

# For 4 robots
num_robots:=4
```

### Robot Starting Positions

Defined in `multibot_gazebo.launch.py`:

| Robot  | Position (x, y) | Orientation (yaw) |
|--------|-----------------|-------------------|
| TB3_0  | (-2.0, -0.5)   | 0.0 rad          |
| TB3_1  | (0.0, 0.5)     | 0.0 rad          |
| TB3_2  | (-1.0, -1.5)   | 0.0 rad          |
| TB3_3  | (2.0, 0.0)     | 0.0 rad          |

### Waypoints

Defined in `track_multibot.py` (lines 361-393):

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

## 📊 Data Analysis

### Automatic Data Logging

When running `pose_eval_coloc`, data is automatically saved to:

```
~/ids_roswk/evaluation_results/multibot/coloc/
```

Files generated:
- `tb3_X_coloc_eval_TIMESTAMP.csv` - Raw evaluation data
- `tb3_X_coloc_statistics_TIMESTAMP.txt` - Summary statistics

### Visualizing Results

Use the data processing script to generate plots:

```bash
cd ~/ids_roswk
python3 src/localization_evaluation/localization_evaluation/data_processing_coloc.py \
    --timestamp 20260107_164136 \
    --num-robots 3
```

**Plots generated:**

1. `trajectory_comparison` - Ground truth vs estimated trajectories
2. `position_error_comparison` - Position error over time
3. `yaw_error_comparison` - Orientation error over time
4. `statistics_comparison` - Bar charts of RMSE, mean, max errors
5. `error_distribution` - Histograms of error distributions
6. `xy_error_scatter` - Spatial distribution of errors

**Output location:**

```
~/ids_roswk/evaluation_results/multibot/coloc/plots/
```

## 📁 Project Structure

```
ids_roswk/
├── src/localization_evaluation/
│   ├── launch/
│   │   ├── multibot_gazebo.launch.py       # Gazebo simulation
│   │   ├── amcl_multibot.launch.py         # AMCL localization
│   │   └── decentralized_coloc.launch.py   # Collaborative localization
│   ├── localization_evaluation/
│   │   ├── track_multibot.py               # Robot controller
│   │   ├── pose_eval_coloc.py              # Evaluation node
│   │   ├── decentralized_coloc_agent.py    # Coloc agent
│   │   ├── pathplan.py                     # RRT path planner
│   │   └── data_processing_coloc.py        # Data visualization
│   ├── param/
│   │   ├── nav2_params_tb3_0.yaml          # AMCL parameters
│   │   ├── nav2_params_tb3_1.yaml
│   │   ├── nav2_params_tb3_2.yaml
│   │   └── nav2_params_tb3_3.yaml
│   ├── models/
│   │   ├── tb3_1/model.sdf                 # Robot models
│   │   ├── tb3_2/model.sdf
│   │   └── tb3_3/model.sdf
│   └── maps/
│       └── map.yaml                        # Environment map
└── evaluation_results/                     # Output data
```

## 🔬 Technical Details

### Collaborative Localization Algorithm

The system implements a decentralized consensus-based collaborative localization using the formula:

```
x̂ᶜᵢ(t) = x̂ᵢ(t) + Σⱼ∈Nᵢ Kᵢⱼ(x̂ⱼ(t) − x̂ᵢ(t))
```

Where:

- `x̂ᶜᵢ(t)` - Collaborative pose estimate for robot i
- `x̂ᵢ(t)` - Local AMCL pose estimate
- `Kᵢⱼ` - Kalman gain (dynamically computed based on covariance)
- `Nᵢ` - Set of neighboring robots

### Communication Protocol

- **Architecture**: Fully connected graph (all robots communicate with each other)
- **Protocol**: Gossip-based consensus
- **Topics**: `/tb3_X/coloc_pose` and `/tb3_X/coloc_belief`
- **Update rate**: ~10 Hz

## 🐛 Troubleshooting

### Issue: Robots not moving

**Cause**: Terminals not started in correct order or AMCL not initialized.

**Solution**: 

1. Wait for "Managed nodes are active" in Terminal 2
2. Ensure Terminal 3 is running before Terminal 4
3. Start Terminal 5 last

### Issue: Large localization errors

**Cause**: AMCL particle filter not converged yet.

**Solution**: Wait 10-15 seconds after starting Terminal 5 for particles to converge.

### Issue: Robot collisions with obstacles

**Cause**: Robot radius parameter too small in path planner.

**Solution**: Increase `ROBOT_RADIUS` in `pathplan.py` (line 91):

```python
ROBOT_RADIUS = 0.20  # Increase from 0.12 to 0.20
```

### Issue: "File not found" errors

**Cause**: Package not built or environment not sourced.

**Solution**:

```bash
cd ~/ids_roswk
colcon build --packages-select localization_evaluation
source install/setup.bash
```
