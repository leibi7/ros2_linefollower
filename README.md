# ROS2 Line Follower (Docker + Gazebo + noVNC)

Teljesen konténeresített ROS 2 Humble demó: Gazebo Classic világ piros pályával és zöld célmezővel, differenciálhajtású robot lefelé néző kamerával, OpenCV-alapú vonalkövető és célfigyelő csomópont, valamint noVNC asztal a Gazebo klienshez (`http://localhost:8080`).

## Előfeltételek
- Docker Desktop / Docker Engine + Docker Compose (macOS vagy Linux)
- Egyszeri internetelérés az első buildhez

## Gyors indítás
```bash
Klónozott repo mappában:
docker compose up -d --build   # sim + control + noVNC
# böngésző: http://localhost:8080 -> Gazebo kliens
```
Működés: a robot a piros sáv elejéről indul, a lefelé néző kamera alapján követi a vonalat, és megáll, amikor a zöld célmezőt eléri. Az állapot a `/goal_reached` topicra kerül, a logok a `logs/run_*` könyvtárba íródnak.

## Szolgáltatások (docker-compose)
- `sim`: `gzserver` a saját világgal; a robotot `spawn_entity.py` illeszti be.
- `control`: OpenCV-alapú vonalkövető és célfigyelő csomópontok.
- `novnc`: könnyű asztal + Gazebo kliens noVNC-n keresztül (`http://localhost:8080`).

Mindegyik konténer `ROS_DOMAIN_ID=23`-at használ, a logok a `./logs` mappába vannak kötve.

## Csomagok (ros2_ws/src)
- `line_follower_world`: Gazebo világ (piros pálya, zöld célmező), `world.launch.py`.
- `line_follower_robot`: URDF/xacro diff-drive robot lefelé néző kamerával + spawn launch.
- `line_follower_control`: vonalkövető (HSV küszöb + PD kormányzás) és célfigyelő csomópont.
- `line_follower_bringup`: fő launch a világ indításához, robot spawnhoz, vezérléshez.

## Hasznos parancsok
- Build (újrafordítás): `docker compose up -d --build`
- Indítás: `docker compose up -d`
- Témák listázása a szimulátorban:  
  `docker exec sim bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && ros2 topic list"`
- Logok: `ls logs/run_*`, majd `line_follower_*.log`, `goal_monitor_*.log`
- Cél kézi trigger:  
  `docker exec control bash -lc "source /opt/ros/humble/setup.bash && source /opt/ros2_ws/install/setup.bash && ros2 topic pub --once /goal_reached std_msgs/Bool '{data: true}'"`


## Megjegyzések
- Gazebo factory plugin engedélyezve, így a robotot `spawn_entity.py` illeszti a `gzserver`-be.
- A célfigyelő `/odom` alapján jelez (alapértelmezés: x=7.0, y=-0.8, tolerancia=0.25 m).
- noVNC port: 8080 (módosítható a `docker-compose.yml`-ben).
