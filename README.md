# ROS2 Line Follower (Docker + Gazebo + noVNC)

End-to-end ROS 2 Humble line-follower demo fully containerized. The stack brings up Gazebo with a red, curved track, a small diff-drive robot with a downward camera, a line-following controller (OpenCV), a goal monitor, and a noVNC desktop at `http://localhost:8080` to watch the simulation.

## Prerequisites
- macOS (or Linux) with Docker Desktop / Docker Engine + Docker Compose
- Internet access for initial image builds

## Quick Start
```bash
git clone <your repo url> ros2_line_follower_docker
cd ros2_line_follower_docker
docker compose build      # builds ROS workspace and noVNC image
docker compose up         # sim + control + noVNC
# open http://localhost:8080 to see Gazebo client
```
What to expect: the robot spawns at the start of a red S-shaped line, detects the line from its downward camera, follows it, and stops once the goal zone (green square) near the end is reached. The goal status is published on `/goal_reached` and logged under `logs/run_*`.

## Services (docker-compose)
- `sim`: runs Gazebo (`gzserver`) with the custom world, spawns the robot.
- `control`: runs the OpenCV-based line follower and the goal monitor.
- `novnc`: provides a lightweight desktop + Gazebo client over noVNC at `http://localhost:8080`.

All containers share `ROS_DOMAIN_ID=23` for DDS discovery and mount `./logs` to persist run logs.

## Packages (ros2_ws/src)
- `line_follower_world`: Gazebo world with red curved track and goal zone.
- `line_follower_robot`: URDF/xacro diff-drive robot with downward camera + spawn launch.
- `line_follower_control`: line follower node (OpenCV HSV threshold + PD steering) and goal monitor node.
- `line_follower_bringup`: main launch to start world, spawn robot, and optionally start control.

## Useful Commands
- Build images: `docker compose build`
- Start full stack: `docker compose up`
- Headless test loop: `bash scripts/run_and_test.sh` (waits, stops, analyzes logs)
- Check logs: `ls logs/run_*` then inspect `line_follower_*.log`, `goal_monitor_*.log`
- Re-run controller only: `docker compose up control`

## Tuning
Parameters for the controller (KP/KD, HSV thresholds, scan rows) are declared in `line_follower_control/line_follower_node.py`. Adjust and rebuild the ROS image (`docker compose build`) to apply changes.

## Log Analysis
`scripts/analyze_logs.py` reads the latest `logs/run_*` folder and returns SUCCESS if the line was detected and the goal was reached; otherwise it reports the failure reason. `scripts/run_and_test.sh` wires this into an automated smoke test.

## Notes
- Gazebo factory plugin is enabled to allow `spawn_entity.py` to insert the robot into `gzserver`.
- The goal monitor uses `/odom` to detect proximity (default goal at `x=7.2`, `y=-0.4`, tolerance `0.35 m`).
- noVNC uses port `8080`; adjust in `docker-compose.yml` if needed.
