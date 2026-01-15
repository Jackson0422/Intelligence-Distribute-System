# Decentralized Collaborative Localization System Usage Guide

## 📋 System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Gazebo simulation environment                │
│              (multibot_gazebo.launch.py)                        │
│                                                                 │
│    tb3_1 (/odom, /scan)        tb3_2 (/tb3_2/odom, /scan)        │
└────────┬────────────────────────────────┬───────────────────────┘
         │                                │
         ▼                                ▼
┌────────────────────┐           ┌────────────────────┐
│   AMCL (tb3_1)     │           │   AMCL (tb3_2)     │
│  /amcl_pose        │           │  /tb3_2/amcl_pose  │
└────────┬───────────┘           └───────┬────────────┘
         │                               │
         ▼                               ▼
┌────────────────────┐           ┌────────────────────┐
│ Collaborative agent│◄─ Gossip ─►│ Collaborative agent│
│ (tb3_1)            │    P2P    │ (tb3_2)            │
│ /coloc_belief      │           │ /tb3_2/coloc_belief│
│ /coloc_pose        │           │ /tb3_2/coloc_pose  │
└────────┬───────────┘           └───────┬────────────┘
         │                               │
         └───────────────┬───────────────┘
                         ▼
                  ┌─────────────────┐
                  │ Performance     │
                  │ evaluation node │
                  │ (pose_eval_coloc)│
                  └─────────────────┘
```

## 🚀 Quick Start Guide

### Option 1: Baseline AMCL Localization

Baseline performance for comparison.

```bash
# Terminal 1: start Gazebo simulation
cd ~/ids_roswk && source install/setup.bash
ros2 launch localization_evaluation multibot_gazebo.launch.py

# Terminal 2: start AMCL localization
cd ~/ids_roswk && source install/setup.bash
ros2 launch localization_evaluation amcl_multibot.launch.py

# Terminal 3: start performance evaluation
cd ~/ids_roswk && source install/setup.bash
ros2 run localization_evaluation pose_eval_multibot

# Terminal 4: start trajectory tracking
cd ~/ids_roswk && source install/setup.bash
ros2 run localization_evaluation track_multibot

# After running for 2-3 minutes, Ctrl+C to stop all nodes
# Results are saved in: evaluation_results/multibot/
```

### Option 2: Collaborative AMCL Localization

Enhanced version with the decentralized collaborative layer.

```bash
# Terminal 1: start Gazebo simulation
cd ~/ids_roswk && source install/setup.bash
ros2 launch localization_evaluation multibot_gazebo.launch.py

# Terminal 2: start AMCL localization
cd ~/ids_roswk && source install/setup.bash
ros2 launch localization_evaluation amcl_multibot.launch.py

# Terminal 3: start the collaborative localization layer (new)
cd ~/ids_roswk && source install/setup.bash
ros2 launch localization_evaluation decentralized_coloc.launch.py

# Terminal 4: start collaborative localization evaluation
cd ~/ids_roswk && source install/setup.bash
ros2 run localization_evaluation pose_eval_coloc

# Terminal 5: start trajectory tracking
cd ~/ids_roswk && source install/setup.bash
ros2 run localization_evaluation track_multibot

# After running for 2-3 minutes, Ctrl+C to stop all nodes
# Results are saved in: evaluation_results/multibot/coloc/
```

## 📊 View and Analyze Results

### Baseline results (AMCL)
```bash
# View the latest statistics reports
cd ~/ids_roswk/evaluation_results/multibot
ls -lht tb3_*_statistics_*.txt | head -2
cat tb3_1_statistics_[latest_timestamp].txt
cat tb3_2_statistics_[latest_timestamp].txt

# Generate plots
cd ~/ids_roswk && source install/setup.bash
python3 src/localization_evaluation/localization_evaluation/data_processing_multibot.py [timestamp]

# Plots are saved in: evaluation_results/multibot/plots/
```

### Collaborative results (Collaborative AMCL)
```bash
# View collaborative localization statistics reports
cd ~/ids_roswk/evaluation_results/multibot/coloc
ls -lht tb3_*_coloc_statistics_*.txt | head -2
cat tb3_1_coloc_statistics_[latest_timestamp].txt
cat tb3_2_coloc_statistics_[latest_timestamp].txt
```

## 🔧 Parameter Tuning

### Collaborative localization parameters

Edit `decentralized_coloc.launch.py` or pass parameters on the command line:

```bash
ros2 launch localization_evaluation decentralized_coloc.launch.py \
    gossip_rate:=2.0 \
    self_weight:=0.6 \
    peer_timeout:=5.0 \
    correction_threshold:=0.02
```

**Parameter descriptions:**

| Parameter | Default | Description |
|------|--------|------|
| `gossip_rate` | 1.0 Hz | Gossip update rate |
| `self_weight` | 0.7 | Weight of the robot's own AMCL estimate (0-1) |
| `peer_timeout` | 3.0 s | Neighbor timeout |
| `correction_threshold` | 0.01 m | Minimum correction threshold |

**Tuning tips:**
- Higher `self_weight` means trusting your own AMCL estimate more.
- Higher `gossip_rate` improves real-time collaboration but increases communication overhead.
- If sensor noise is large, reduce `self_weight`.
- If the network is unstable, increase `peer_timeout`.

## 📈 Expected Performance Comparison

### Baseline (AMCL)
- **Position RMSE**: about 6-8 cm
- **Yaw RMSE**: about 5-8°
- **Features**: independent localization, no collaboration

### Collaborative (Collaborative AMCL)
- **Position RMSE**: about 4-6 cm (20-30% improvement)
- **Yaw RMSE**: about 3-5° (30-40% improvement)
- **Features**:
  - ✓ Decentralized P2P communication
  - ✓ Multi-source information fusion
  - ✓ Faster convergence
  - ✓ Stronger robustness

## 🔍 Debugging and Monitoring

### View live topics
```bash
# View AMCL poses
ros2 topic echo /amcl_pose --once
ros2 topic echo /tb3_2/amcl_pose --once

# View collaborative localization poses
ros2 topic echo /coloc_pose --once
ros2 topic echo /tb3_2/coloc_pose --once

# View belief broadcasts (JSON format)
ros2 topic echo /coloc_belief
ros2 topic echo /tb3_2/coloc_belief
```

### View node status
```bash
# List all running nodes
ros2 node list | grep -E "amcl|coloc"

# View collaborative agent logs
ros2 run localization_evaluation decentralized_coloc_agent --ros-args --log-level debug
```

## 🧪 Experiment Suggestions

### Comparison experiment steps

1. **First run - Baseline**
   - Run baseline AMCL for 2-3 minutes.
   - Record the timestamp.
   - Stop all nodes.

2. **Second run - Collaborative**
   - Fully restart the system (close Gazebo).
   - Run AMCL + collaboration layer for 2-3 minutes.
   - Record the timestamp.
   - Stop all nodes.

3. **Comparison analysis**
   - Compare RMSE between the two runs.
   - Compute performance improvement percentage.
   - Analyze differences in error distributions.

### Experiment notes

⚠️ **Before each experiment:**
1. Fully close Gazebo.
2. Clean ROS processes: `pkill -9 -f "gazebo|ros2"`
3. Restart all nodes.
4. Ensure robots start from the same initial positions.

## 🎓 Course Project Report Highlights

Recommended topics for the "Distributed Intelligent Systems" course report:

### 1. System design
- Decentralized architecture (no central node)
- P2P communication mechanism (ROS2 topics)
- Gossip protocol implementation
- Weighted consensus algorithm

### 2. Key techniques
- **Distributed consensus**: each robot maintains its own belief, reaching consensus via Gossip.
- **Fault tolerance**: neighbor timeout mechanism; one robot failure does not affect others.
- **Scalability**: easy to extend to N robots.
- **Real-time**: 1 Hz update rate.

### 3. Performance evaluation
- Compare localization accuracy of Baseline vs Collaborative.
- Analyze the performance gains from collaboration.
- Discuss the impact of weight parameters on performance.
- Evaluate communication overhead.

### 4. Future improvements
- Dynamic weight adjustment (adaptive by covariance)
- More advanced fusion algorithms (Kalman filtering)
- Multi-hop communication (indirect neighbors)
- Heterogeneous robot collaboration

## ❓ FAQ

**Q: The collaborative localization node starts but prints no output?**
A: Make sure AMCL is running and publishing poses. The collaborative agent needs AMCL poses to start working.

**Q: I see "neighbor timeout" warnings?**
A: Check whether the other robot's collaborative agent is running, or increase the `peer_timeout` parameter.

**Q: The error gets worse after collaboration?**
A: `self_weight` may be too low. Try increasing it to 0.8 to trust your own AMCL estimate more.

**Q: How do I scale to 3 or more robots?**
A: Modify the launch file, add more `agent` nodes, and set `peer_ids` correctly.

## 📞 Support

Having issues? Check the following:
1. ROS2 environment is sourced correctly.
2. All required nodes are running.
3. Topic connections are healthy (`ros2 topic list`).
4. Logs show no errors.

Good luck with the experiments! 🎉
