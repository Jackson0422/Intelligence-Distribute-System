#

## Installation

### 1. Clone the Repository

```bash
git clone <repository-url>
```

### 2. Build the Workspace

```bash
cd ~/ids_roswk
colcon build --symlink-install
source install/setup.bash
```

### 3. Set Environment Variables

Add to your `~/.bashrc`:

```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ids_roswk/src/localization_evaluation/models
source ~/ids_roswk/install/setup.bash
```

## Quick Start

### Running the Complete System (3 Robots)

You need **5 terminals** to run the full collaborative localization system.

#### Terminal 1: Launch Gazebo Simulation

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=3
```

**Wait until all robots are spawned in Gazebo before proceeding.**

#### Terminal 2: Launch AMCL Localization

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=3
```

**Wait for "Managed nodes are active" message.**

#### Terminal 3: Launch Collaborative Localization

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=3
```

#### Terminal 4: Launch Evaluation Node

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 run localization_evaluation pose_eval_coloc --ros-args -p num_robots:=3
```

#### Terminal 5: Launch Robot Controllers

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=3
```

**The robots will now navigate through their predefined waypoints.**

```bash
pkill -9 gzserver && pkill -9 gzclient
source ~/.bashrc
export LIBGL_ALWAYS_SOFTWARE=1
export TURTLEBOT3_MODEL=burger
```