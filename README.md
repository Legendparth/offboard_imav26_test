# drone_testing

ROS 2 (Humble) package for autonomous offboard flight on a PX4 vehicle, running
on a Jetson companion computer.

Airframe this is written for: **Pixhawk 6C** (internal IMUs) + **ARK Flow**
(optical flow + 1-D distance sensor). There is **no GPS and no external
position source** in the takeoff test — height comes from the rangefinder and
lateral position from optical flow, and the code is written around the
limitations that implies.

The headline node is `offboard_takeoff`: arm → sit on the ground → climb to a
set altitude → hold → descend → disarm, entirely on its own.

---

## 1. Prerequisites

### Workspace layout

```
~/px4_ros_ws/
└── src/
    ├── px4_msgs/                 # must match your PX4 firmware version
    ├── px4_ros_com/
    └── offboard_imav26_test/     # this package (ROS package name: drone_testing)
```

### System packages

```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions
sudo apt install ros-humble-micro-ros-agent      # or build micro-XRCE-DDS-Agent from source
pip3 install pyserial pymavlink
```

### Serial port permissions

The Pixhawk is on `/dev/ttyTHS1` on the Jetson. You need to be in `dialout`:

```bash
sudo usermod -aG dialout $USER   # log out and back in (or reboot)
```

Nothing else may hold that port. Jetsons often bind a serial console to it:

```bash
systemctl status serial-getty@ttyTHS1
sudo systemctl disable --now serial-getty@ttyTHS1
```

### PX4 side

On the flight controller, the port wired to the Jetson must be running the
uXRCE-DDS client at a matching baud rate (921600 here):

- `UXRCE_DDS_CFG` → the TELEM port you are using
- `SER_TEL2_BAUD` (or whichever port) → 921600

---

## 2. Build

```bash
cd ~/px4_ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_testing
source install/setup.bash
```

> **Do not use `--symlink-install` on this machine.** setuptools ≥ 80 removed
> `develop --uninstall`, which colcon calls when cleaning a previous symlink
> install, and the build fails with `option --uninstall not recognized`. If you
> already hit it:
>
> ```bash
> rm -rf ~/px4_ros_ws/build/drone_testing ~/px4_ros_ws/install/drone_testing
> colcon build --packages-select drone_testing
> ```

Add the sourcing to your shell so every new terminal has it:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/px4_ros_ws/install/setup.bash' >> ~/.bashrc
```

---

## 3. Verify the link before you fly

Start the DDS agent on its own and confirm PX4 topics appear.

**Terminal 1 — agent only (this is the default):**

```bash
ros2 launch drone_testing takeoff_test.launch.py
```

**Terminal 2 — check:**

```bash
ros2 topic list | grep /fmu/
ros2 topic echo /fmu/out/vehicle_status_v1 --once
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

In that last message you want to see, **props off, on the ground**:

| field              | expected                                            |
|--------------------|-----------------------------------------------------|
| `z_valid`          | `true` — no height estimate, no flight               |
| `dist_bottom_valid`| `true` — the rangefinder is being fused              |
| `dist_bottom`      | roughly your actual height above the floor           |
| `xy_valid`         | may be `false` on the ground; that is normal for flow |

`offboard_takeoff` refuses to arm without `z_valid` **and** `dist_bottom_valid`.
If they are false, fix the sensor before going further — the log line
`Not arming: need z_valid and dist_bottom_valid` is telling you the truth.

---

## 4. Run the autonomous takeoff

### The recommended way (keyboard aborts stay live)

Run the agent from the launch file and the flight node **by hand in a second
pane**. Launching the node through `ros2 launch` means its stdin is not a tty,
which kills the `q` / `k` keyboard aborts.

**Pane 1:**

```bash
ros2 launch drone_testing takeoff_test.launch.py
```

**Pane 2:**

```bash
ros2 run drone_testing offboard_takeoff --ros-args \
  -p takeoff_altitude:=0.30 \
  -p hold_seconds:=5.0 \
  -p ground_wait_seconds:=5.0 \
  -p climb_speed:=0.35 \
  -p land_speed:=0.15 \
  -p request_offboard_from_ros:=true
```

**Start low.** 0.30 m for the first flight, then work up.

### Keyboard aborts (only when run as above)

| key | effect                                                             |
|-----|--------------------------------------------------------------------|
| `q` | abort into a controlled descent from wherever it is                 |
| `k` | force-disarm **immediately** — motors cut, the vehicle drops        |

Your **RC kill switch is the real safety net**. The keyboard is a convenience.
Flipping the TX out of Offboard also makes the node stand down and let go.

### Everything from one launch file

If you accept losing the keyboard aborts:

```bash
ros2 launch drone_testing takeoff_test.launch.py \
  agent_only:=false \
  takeoff_altitude:=0.30 \
  hold_seconds:=5.0
```

### Launch arguments

| argument                    | default  | meaning                                                        |
|-----------------------------|----------|----------------------------------------------------------------|
| `agent_only`                | `true`   | `true` = start only the DDS agent (+ LCD); run the node by hand |
| `takeoff_altitude`          | `0.80`   | metres above the arming point                                   |
| `hold_seconds`              | `15.0`   | station-keeping time once the altitude is reached               |
| `ground_wait_seconds`       | `5.0`    | armed on the ground before the climb starts                     |
| `climb_speed`               | `0.35`   | m/s the climb setpoint ramps at                                 |
| `land_speed`                | `0.15`   | m/s the descent setpoint ramps at                               |
| `request_offboard_from_ros` | `true`   | `false` = you flip the Offboard switch on the TX yourself        |
| `lcd`                       | `true`   | start the Arduino LCD status node                               |
| `lcd_port`                  | `''`     | Arduino serial port; empty = auto-detect `ttyACM*` / `ttyUSB*`   |

With `request_offboard_from_ros:=false` the node waits **indefinitely** for you
to flip the Offboard switch, so it is safe to start it long before you are
ready to fly.

---

## 5. What the flight actually does

```
PREPARATION      stream setpoints, wait for a healthy z + rangefinder estimate
OFFBOARD_REQUEST enter Offboard (from ROS, or wait for your TX switch)
ARMING           arm, and latch the height datum
GROUND_WAIT      sit armed, setpoint pressed 0.15 m *below* ground so it stays planted
TAKEOFF          ramp the z setpoint up to the target
HOLD             station-keep; latch x/y position hold once optical flow is healthy
LANDING          ramp back down, overshooting 0.5 m below ground
DISARMING        disarm once the land detector confirms touchdown
DONE             stop streaming setpoints and let go of the aircraft
```

Two things worth knowing about why it is written this way:

- **Horizontal is flown as a zero-velocity setpoint, not a position setpoint,**
  for takeoff and landing. On a flow-only airframe the x/y estimate near the
  ground is dead-reckoned garbage; a latched position would be flown out the
  moment flow started correcting it. Position hold is latched only once
  airborne with healthy flow, onto a *fresh* estimate.
- **Arrival at altitude requires three independent agreements** — the land
  detector says airborne, the EKF-relative altitude is in band, and the
  rangefinder roughly agrees. An EKF height reset alone can otherwise make the
  node "arrive" while still sitting on the ground.

### Reading the status output

`offboard_takeoff` publishes `/takeoff_status` as one pipe-separated line:

```
STAGE|ARM|altitude|xy-mode|detail
```

where `xy-mode` is `POS` (position hold latched), `FLO` (flow healthy, still on
velocity hold) or `---` (no usable flow).

```bash
ros2 topic echo /takeoff_status
```

---

## 6. Optional: LCD status display

An Arduino running `arduino/tft_status/tft_status.ino` shows the stage, arm
state and altitude. It is started by default with the launch file:

```bash
ros2 launch drone_testing takeoff_test.launch.py lcd:=true lcd_port:=/dev/ttyACM0
```

Disable it with `lcd:=false`.

---

## 7. Optional: start at boot via systemd

`drone_testing/px4-agent.service` brings the Jetson up flight-ready: DDS agent
plus the takeoff node waiting for your Offboard switch.

```bash
sudo cp ~/px4_ros_ws/src/offboard_imav26_test/drone_testing/px4-agent.service \
        /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now px4-agent.service
```

```bash
systemctl status px4-agent.service
journalctl -u px4-agent.service -f          # the flight log
sudo systemctl stop px4-agent.service       # abort from a shell
sudo systemctl disable --now px4-agent.service
```

**Read this before enabling it:**

- systemd gives the node no tty, so `q` / `k` are **dead**. Your RC kill switch
  and the TX mode switch are the only aborts.
- It runs on **every** boot — including a battery swap in the field or a
  brownout reboot. The drone is armed-and-waiting whenever it is powered.
- `Restart=no` is deliberate. `on-failure` would relaunch a node that had
  aborted and let it re-arm on its own.

Altitude and hold time live in the `ExecStart=` line; after editing:

```bash
sudo systemctl daemon-reload && sudo systemctl restart px4-agent.service
```

---

## 8. Other nodes in the package

| node               | what it does                                                     |
|--------------------|------------------------------------------------------------------|
| `offboard_takeoff` | the autonomous takeoff / hold / land test (this README's subject) |
| `offboard_mission` | multi-waypoint offboard mission                                   |
| `zed_localization` | feeds ZED visual odometry into PX4 as `vehicle_visual_odometry`   |
| `lcd_status`       | drives the Arduino status display                                 |
| `pixhawk_node`     | MAVLink telemetry reader                                          |
| `cam`              | camera capture helper                                             |

Other launch files:

- `arm_test.launch.py` — agent + `offboard_mission`, for arm/disarm bench tests
- `offboard_launch.launch.py` — agent + ZED localization + `offboard_mission`

---

## 9. Troubleshooting

| symptom | cause / fix |
|---|---|
| `Waiting for VehicleStatus from PX4...` forever | DDS link down. Check the agent is running, the baud is 921600 both ends, `UXRCE_DDS_CFG` is set, and nothing else holds `/dev/ttyTHS1`. |
| `Not arming: need z_valid and dist_bottom_valid` | Rangefinder not being fused. Check the ARK Flow wiring and `EKF2_HGT_REF` / `EKF2_RNG_CTRL`. |
| `Offboard mode not entered in time` | PX4 rejected the mode. Usually pre-arm checks failing — look at the PX4 console or QGC for the reason. |
| `Arming rejected / timed out` | Pre-arm check failure, or the safety switch is not pressed. |
| `Altitude disagreement: ekf=... lidar=...` | The EKF datum and the rangefinder disagree by more than 32 cm. Normally an estimator reset mid-climb; the node correctly refuses to accept arrival. |
| `Offboard lost; PX4 has control now` | The TX switch moved, or PX4 failsafed. The node lets go on purpose. |
| `error: option --uninstall not recognized` on build | Stale `--symlink-install` state; see the build section above. |
| `q` / `k` do nothing | The node was started via `ros2 launch` or systemd, so stdin is not a tty. Run it with `ros2 run` in its own pane. |

---

## 10. Pre-flight checklist

1. Props **off** for the first run of any changed code.
2. `ros2 topic echo /fmu/out/vehicle_local_position_v1 --once` → `z_valid` and
   `dist_bottom_valid` both true.
3. RC kill switch tested on the bench, this session.
4. `takeoff_altitude` set low (0.30 m).
5. Clear space around and above the vehicle — flow-only hold drifts.
6. You know which pane has the `q` key.
