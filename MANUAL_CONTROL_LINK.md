# Manual Control Link (Operator Override)

This document explains the manual control link added for emergency/operator takeover.

## Goal

Allow the operator to switch from autonomous mission control to manual control at any time from the computer keyboard, and switch back to autonomous mode when needed.

## Link Design

The control link is built as ROS 2 topics:

- `/manual_mode` (`std_msgs/Bool`)
  - `true`: operator/manual control has priority
  - `false`: autonomous mission regains control
- `/manual_cmd_vel` (`geometry_msgs/Twist`)
  - operator velocity commands (x, y, z, yaw)
- `/cmd_vel` (`geometry_msgs/Twist`)
  - final command sent by mission node to Gazebo controller

### Arbitration logic

Implemented in `scripts/slam_boundary_mapping_mission.py`:

- If manual mode is enabled, mission node immediately publishes manual velocity to `/cmd_vel`.
- Autonomous FSM is bypassed while manual mode is active.
- If manual command stream is stale (timeout), mission node sends zero velocity for safety.
- When manual mode is disabled, control returns to autonomous mission state machine.

## Files Added/Updated

- Added keyboard link node:
  - `scripts/manual_control_link.py`
- Updated mission node to support manual override:
  - `scripts/slam_boundary_mapping_mission.py`
- Updated launch to run mission in sim time (important for SLAM map frame):
  - `launch/slam_arena_mission.launch.py`

## Run Procedure

## 1) Start simulation and mission

Terminal 1:

```bash
cd /home/kunal/Desktop/drone
HOME=/tmp gz sim -r worlds/arena.sdf
```

Terminal 2:

```bash
cd /home/kunal/Desktop/drone
source /opt/ros/jazzy/setup.bash
HOME=/tmp ros2 launch /home/kunal/Desktop/drone/launch/slam_arena_mission.launch.py
```

## 2) Start manual control link (operator terminal)

Terminal 3:

```bash
cd /home/kunal/Desktop/drone
source /opt/ros/jazzy/setup.bash
python3 /home/kunal/Desktop/drone/scripts/manual_control_link.py
```

## Keyboard Map

Mode keys:
- `m`: enable MANUAL mode immediately
- `n`: disable MANUAL mode (return to autonomous)

Motion keys (incremental setpoints):
- `w` / `s`: forward / backward (`x`)
- `a` / `d`: left / right (`y`)
- `i` / `k`: up / down (`z`)
- `j` / `l`: yaw left / yaw right (`angular.z`)

Utility keys:
- `space`: zero all velocities
- `r`: print help in terminal
- `q`: exit manual link and return to autonomous mode

## Safety Behavior

- Manual mode can be activated at any time.
- If manual command input pauses for too long, command is forced to zero by timeout.
- Exiting manual link (`q` or Ctrl+C) sends mode false and zero command before shutdown.

## Notes

- Mission launch now sets `use_sim_time:=true` for the mission node, which is required for consistent TF with SLAM and for the `map` frame to appear correctly in RViz.
