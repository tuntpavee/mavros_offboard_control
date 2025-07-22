# MAVROS Offboard Control

High‑level node that streams **`TrajectorySetpoint`** to PX4 via MAVROS + micro‑ROS.  
Works on both **real hardware** (Pixhawk ↔ STM32 XRCE‑DDS bridge) and **Gazebo SITL**.

---

## 1. Prerequisites

| Package / Tool       | Tested Version     | Notes                                          |
|----------------------|--------------------|------------------------------------------------|
| **PX4 Firmware**     | `main` (Jul 2025)  | Gazebo Classic + Ignition                      |
| **ROS 2**            | Humble Hawksbill   | Desktop + Dev tools                            |
| **MAVROS 2**         | 2025.5.5           | `apt install ros-humble-mavros*`               |
| **micro-ROS agent**  | 2.1                | `sudo apt install microxrcedds-agent`          |
| **QGroundControl**   | v4.3+              | For parameter sync / log download              |
| **px4_msgs**         | `main`             | `git clone https://github.com/PX4/px4_msgs.git`|
---

## 2. Build the Package

```bash
# In the workspace root
colcon build --symlink-install --packages-select px4_offboard_control
source install/setup.bash
```

> **Note:**  
> If you see `include/...` not found, comment out the `install(DIRECTORY include/...)`  
> line in `px4_offboard_control/CMakeLists.txt`.

---

## 3. Running on Real Hardware

### Terminal 1: MAVROS
```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=/dev/ttyACM0:115200 \     # Pixhawk USB CDC
  -p gcs_url:=udp://@ \                 # (optional) forward to QGC on same PC
  -p target_system_id:=1               # usually 1 for Pixhawk
```

### Terminal 2: micro-ROS Agent (STM32 side)
```bash
sudo MicroXRCEAgent serial -D /dev/ttyUSB0
```

> Adjust `/dev/ttyUSB0` if needed.  
> **Tip:** Add yourself to the `dialout` group so you don’t need `sudo`:
```bash
sudo usermod -aG dialout $USER && newgrp dialout
```

---

## 4. Running in Simulation (SITL + Gazebo)

### Terminal 1: MAVROS (offboard setpoint bridge)
```bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14540@ \          # listen for PX4 SITL UDP
  -p gcs_url:=udp://@14550             # forward to QGC on port 14550
```

### Terminal 2: micro-ROS Agent (simulated DDS peer)
```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 3: Launch PX4 SITL in Gazebo Classic
```bash
cd PX4-Autopilot
make px4_sitl gz_x500_baylands
```

### Optional: Launch QGroundControl
```bash
cd ~/Downloads
./QGroundControl-x86_64.AppImage
```

---

## 5. Running the Offboard Node

In any new terminal (after sourcing setup.bash):

```bash
source ~/YOUR_WS/install/setup.bash
ros2 run px4_offboard_control offboard_control
```

- Listens to `/mavros/local_position/pose` (ENU)
- Publishes NED‑converted `TrajectorySetpoint` to `/fmu/in/trajectory_setpoint`

> It holds for 5 s, captures takeoff offset, climbs to 1.2 m, then follows `_src/path.csv_`.  
> Tune `DIST_THRESHOLD_METERS`, `DWELL_TIME_SEC`, etc. in the source if needed.

---

## 6. Common Issues

| Symptom                                        | Fix                                                                 |
|-----------------------------------------------|----------------------------------------------------------------------|
| can't compare times with different time sources | Ensure all `rclcpp::Time` operations use the same clock or raw nanoseconds |
| Agent prints `Error: bind: Address already in use` | Another Agent is running; kill it or choose a new port/device        |
| QGC shows `Parameters missing`                | Wait 10 s after boot, then sync parameters (🔄)                      |
| Drone doesn’t move in SIM                     | Ensure drone is armed and `/fmu/in/trajectory_setpoint` is streaming ≥ 2 Hz |