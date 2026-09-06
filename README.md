# drone_testing

ROS 2 (Humble) package for autonomous offboard flight on a PX4 vehicle, running
on a Jetson companion computer.

Airframe this is written for: **Pixhawk 6C** (internal IMUs) + **ARK Flow**
(optical flow + 1-D distance sensor). There is **no GPS and no external
position source** in the takeoff test — height comes from the rangefinder and
lateral position from optical flow, and the code is written around the
limitations that implies.

The headline node is `offboard_takeoff`: arm → sit on the ground → climb to a
set altitude → hold → descend → disarm, entirely on its own. `offboard_translate`
adds a horizontal leg to that — climb, then move a set distance forward,
backward, left or right, then land (see section 6). `offboard_sequence` goes one
further: climb, then a list of motions — translations, altitude changes and yaws
— flown one at a time, then land (see section 6b).

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

## 6. The translate test (`offboard_translate`)

Same flight as above with a horizontal leg in the middle: arm → ground wait →
climb to **1.0 m** → hold → **move 1.0 m** in a body-frame direction → hold →
land. Everything about the estimator gating, aborts and landing is identical —
it is the takeoff node plus two stages.

### Running it

**Pane 1:**

```bash
ros2 launch drone_testing translate_test.launch.py
```

**Pane 2:**

```bash
ros2 run drone_testing offboard_translate --ros-args \
  -p takeoff_altitude:=1.0 \
  -p move_distance:=1.0 \
  -p move_direction:=forward \
  -p move_speed:=0.30 \
  -p hold_seconds:=5.0 \
  -p post_hold_seconds:=5.0
```

**Start small.** `takeoff_altitude:=0.5`, `move_distance:=0.5` for the first
flight. A flow-only lateral move needs far more clear floor than a hover does —
give it several metres in the direction of travel and be ready on `q`.

### Directions

`move_direction` is **body frame**, relative to the yaw the vehicle held when it
armed (yaw is pinned for the whole flight — it never rotates):

| value      | where it goes                |
|------------|------------------------------|
| `forward`  | out the nose (default)       |
| `backward` | out the tail                 |
| `left`     | out the left side            |
| `right`    | out the right side           |

### Parameters

| parameter            | default   | meaning                                              |
|----------------------|-----------|------------------------------------------------------|
| `move_distance`      | `1.0`     | metres to travel                                     |
| `move_direction`     | `forward` | `forward` / `backward` / `left` / `right`            |
| `move_speed`         | `0.30`    | m/s the horizontal setpoint is walked at             |
| `takeoff_altitude`   | `1.0`     | metres above the arming point                        |
| `hold_seconds`       | `5.0`     | hold **before** the move — the flow latch happens here |
| `post_hold_seconds`  | `5.0`     | hold **after** the move, before the descent          |

The rest (`ground_wait_seconds`, `climb_speed`, `land_speed`,
`request_offboard_from_ros`, `lcd`, `lcd_port`) are the same as the takeoff
test. `translate_test.launch.py` takes all of them as launch arguments too, with
`agent_only:=true` by default.

### Stages

```
... TAKEOFF, then:
HOLD        station-keep and wait for optical flow to latch x/y position hold
TRANSLATE   walk the held point to the target at move_speed
POST_HOLD   station-keep at the new point
LANDING     as before
```

### Why the move is a position setpoint, not a velocity one

A velocity setpoint is open loop *with respect to distance* — "1 m forward"
becomes "0.3 m/s for 3.3 s and hope", and flow bias, the accel/decel ramps and
any wind integrate straight into the distance actually flown. A position
setpoint closes that loop: the vehicle flies to a point and brakes itself
there, so bias shows up as a bounded offset instead of unbounded drift.

The cost is that a position setpoint is only as good as the x/y estimate it is
written in, so the move is **gated on the flow actually working**:

- the ground and the climb stay on zero-velocity hold, exactly as before;
- x/y position hold is latched only once airborne on a *fresh* estimate;
- only then does the move start, and it moves the **latched point**, walking it
  to the target at `move_speed` — a carrot. That is what sets the flight speed
  (rather than `MPC_XY_VEL_MAX`) and keeps the position error PX4 is correcting
  small the whole way. The carrot is leashed to 0.40 m ahead of the measured
  position so it cannot run away, or drag a snagged vehicle.

If the flow never latches within 15 s, or drops out mid-move, the node
**abandons the move and lands**. It will not dead-reckon the move on velocity:
a move you cannot measure is not a move worth flying.

On completion it logs the distance actually travelled against the distance
commanded — that number is your flow accuracy, and it is worth writing down
after each flight.

### Status output

Same `/takeoff_status` topic and format as the takeoff node, so the LCD works
unchanged. During the move the detail field reads e.g. `for0.62` — direction
plus metres still to go.

---

## 6b. The sequence test (`offboard_sequence`)

Same machinery again, but instead of one horizontal leg it flies **a list of
motions, one at a time**: arm → ground wait → climb → hold → step 1 → settle →
step 2 → settle → step 3 → settle → step 4 → hold → land.

A step is one of:

| step                                 | units   | what it does                          |
|--------------------------------------|---------|---------------------------------------|
| `forward` / `backward` / `left` / `right` | metres  | horizontal translation           |
| `up` / `down`                        | metres  | altitude change from where it is now  |
| `yaw`                                | degrees | rotate in place, `+` = clockwise seen from above |

### Running it

**Pane 1:**

```bash
ros2 launch drone_testing sequence_test.launch.py
```

**Pane 2:**

```bash
ros2 run drone_testing offboard_sequence --ros-args \
  -p takeoff_altitude:=1.0 \
  -p sequence:="forward 1.0, yaw 30, up 0.5, right 1.0"
```

The whole mission is that one `sequence` string: comma-separated items, each a
name and a number separated by a space, a colon or an `=`. Four steps is what
this test was written for; any number up to 12 is accepted. A malformed string
is **fatal at startup** — the node refuses to run rather than fly a mission
other than the one you typed.

**Start small.** `takeoff_altitude:=0.5` and half-metre steps for the first
flight, and check you have clear floor along the *whole* path, not just the
first leg. Be ready on `q`.

### Which frame the directions are in

`forward` means **the direction the vehicle was facing when it armed**, and it
keeps meaning that for the entire flight. A `yaw 30` step rotates the airframe
but does **not** rotate what `forward` means — so in the example above, `right
1.0` after the yaw flies the same ground track it would have flown without the
yaw, with the airframe crabbing 30°.

This is deliberate, and it is the same convention as a velocity setpoint in
SITL: every setpoint that leaves this node is in the NED local frame, not the
body frame, so a fixed reference yaw is the only reading that does not silently
depend on how well the yaw step tracked.

Pass `direction_frame:=current` if you want the other convention, where each
move is resolved against the yaw commanded at that point and the example flies
a 30° dog-leg.

### Parameters

| parameter           | default                                | meaning                                              |
|---------------------|----------------------------------------|------------------------------------------------------|
| `sequence`          | `forward 1.0, yaw 30, up 0.5, right 1.0` | the mission                                        |
| `direction_frame`   | `home`                                 | `home` / `current` — see above                       |
| `step_hold_seconds` | `3.0`                                  | settle time **between** steps                        |
| `yaw_rate`          | `0.35`                                 | rad/s (~20°/s) the yaw setpoint is walked at         |
| `min_altitude`      | `0.4`                                  | m a `down` step may not go below                     |
| `max_altitude`      | `3.0`                                  | m an `up` step may not exceed                        |

`takeoff_altitude`, `hold_seconds`, `post_hold_seconds`, `move_speed`,
`ground_wait_seconds`, `climb_speed`, `land_speed`,
`request_offboard_from_ros`, `lcd`, `lcd_port` are all the same as the translate
test, and `sequence_test.launch.py` takes every one of them as a launch
argument with `agent_only:=true` by default.

### How a step can end without ending the flight

Steps are individually recoverable — a bad one is reported and the sequence
carries on, because the next step may not depend on whatever failed:

| outcome        | when                                                          |
|----------------|---------------------------------------------------------------|
| `done`         | reached and settled inside tolerance                          |
| `SKIPPED`      | a horizontal step, but optical flow never latched x/y         |
| `ABANDONED`    | a horizontal step, flow lost part-way through it              |
| `TIMED OUT`    | did not get there in time; holds wherever it actually is      |

Yaw and altitude steps do **not** need the lateral estimate — they are measured
by the gyro/compass and the lidar — so they still run on a flight where the
flow never comes good and the horizontal steps are skipped. Anything more
serious than a failed step (lost height estimate, rangefinder fusion stopping,
Offboard taken away) lands or stands down exactly as in the other tests.

The per-step results are printed as one summary line at the end of the flight,
and again after disarm.

### Settle time between steps

`step_hold_seconds` exists so each step starts from a **stationary** vehicle.
Without it, step *n+1* samples its start point while the vehicle is still
overshooting step *n*, and the errors compound down the sequence instead of
each step correcting from where the previous one really finished.

### Status output

Same `/takeoff_status` topic and format as the other nodes, so the LCD works
unchanged. The detail field carries the step counter: `2/4 yaw18`, `1/4
for0.62`, `3/4 up1.50`.

---

## 6c. The window scan (`window_detect` + `window_scan`)

Takeoff, sweep the nose through a 90 degree arc until the ZED sees the
window, lock onto it, land 40 s after the climb started.

Two nodes:

| node            | what it does |
|-----------------|--------------|
| `window_detect` | subscribes to the ZED image (and depth) topics from `zed_wrapper`, runs the HSV / quadrilateral window detection, publishes `/window_detected` |
| `window_scan`   | the flight. Everything about arming, the climb, the health gates and the landing is inherited from `offboard_sequence`; only the middle of the flight is different |

### The camera side

`window_detect` reads two topics published by `zed_wrapper`:

```
/zed/zed_node/rgb/image_rect_color      rectified LEFT colour image
/zed/zed_node/depth/depth_registered    depth, 32FC1 in metres, same frame
```

**Check these names on the Jetson first** — they vary between wrapper
versions and with `camera_name`:

```bash
ros2 topic list | grep zed
```

and if yours differ, pass `image_topic:=...` / `depth_topic:=...`. Depth is
optional (`use_depth:=false`): without it the window is still detected, only
the corner distances go missing.

It publishes:

| topic                     | type               | what |
|---------------------------|--------------------|------|
| `/window_detected`        | `std_msgs/Bool`    | debounced: true after 3 consecutive hits, false after 5 misses |
| `/window_info`            | `std_msgs/String`  | `u\|v\|offset\|area\|d1\|d2\|d3\|d4\|dc` — centre pixel, horizontal offset as a fraction of half the frame (-1 left, 0 centred, +1 right), contour area, the four corner depths and the centre depth |
| `/window_detection/image` | `sensor_msgs/Image`| the annotated frame, with a `WINDOW LOCKED` / `searching...` banner |

`window_detect` does **not** use `cv_bridge`. Its conversion lives in a
compiled extension built against the distro's NumPy, and a pip-installed
NumPy 2 in `~/.local` makes it segfault on the first frame (`process has
died ... exit code -11`). The node converts `sensor_msgs/Image` in pure
NumPy instead, so it runs whichever NumPy is on the path.

### Seeing whether the window is detected

In the terminal — the node logs one line a second either way, plus a WARN
the moment the detection latches or is lost:

```
[INFO] [window_detect]: window: no   (streak 12 misses, 143 frames seen)
[WARN] [window_detect]: WINDOW DETECTED  (centre=(671,342) offset=+0.05 area=18422px dist=2.31m)
[INFO] [window_detect]: window: YES  centre=(671,342) offset=+0.05 area=18422px dist=2.31m
```

or straight off the topics:

```bash
ros2 topic echo /window_detected
ros2 topic echo /window_info
ros2 topic hz /window_detection/image      # is the camera actually feeding us?
```

### Watching the feed live while it flies

Raw `bgr8` at 1280x720x15 fps is about 40 MB/s. WiFi will not carry that, so
neither route below ever puts a raw frame on the network — both are fed by
one JPEG encode, downscaled by `stream_scale` (0.5 = quarter the pixels) at
`jpeg_quality` (60). That works out around 10 KB a frame, ~150 KB/s.

**A browser — nothing needed on the viewing machine, not even ROS:**

```
http://<jetson-ip>:8080/
```

The node serves the annotated frame as MJPEG on that port (`/snapshot.jpg`
for a single still). If the Jetson is only reachable through ssh, tunnel it
and open `http://localhost:8080/` on your laptop:

```bash
ssh -L 8080:localhost:8080 ark-jetson-orin@<jetson-ip>
```

`stream_port:=0` turns the server off.

**rqt_image_view over the ROS network** (laptop on the same subnet, same
`ROS_DOMAIN_ID`): open `/window_detection/image` and switch the transport
dropdown to **compressed** — that selects
`/window_detection/image/compressed`, which is the JPEG topic. Do not view
the raw topic over WiFi.

```bash
ros2 run rqt_image_view rqt_image_view
```

On the Jetson itself with a monitor, the raw topic is fine:

```bash
ros2 run rqt_image_view rqt_image_view /window_detection/image
```

With a monitor on the Jetson you can also have the original OpenCV windows
back — the frame and the HSV mask, same as the standalone script:

```bash
ros2 run drone_testing window_detect --ros-args -p show_windows:=true
```

On the **LCD**: `lcd_status` subscribes to `/window_detected` itself and
row 4 becomes `flow ok  win YES` / `win no` / `win --` (the last one means
the detector is not publishing at all). The banner reads `SCANNING` during
the sweep and `WIN LOCK` once it has locked on.

### Running it

Bench test, no props — camera and detection only, no DDS agent and no
flight node. This is how you tune the HSV thresholds:

```bash
ros2 launch drone_testing window_scan.launch.py flight:=false
ros2 launch drone_testing window_scan.launch.py flight:=false publish_mask:=true
```

Flight. The default starts the agent, the ZED and the detector but not the
flight node, so you run that by hand and keep the `q` / `k` aborts:

```bash
ros2 launch drone_testing window_scan.launch.py
ros2 run drone_testing window_scan --ros-args \
    -p takeoff_altitude:=1.0 -p flight_seconds:=40.0
```

Everything from the launch file (no keyboard abort — RC kill switch only):

```bash
ros2 launch drone_testing window_scan.launch.py agent_only:=false
```

Add `zed:=false` if `zed_wrapper` is already running from somewhere else,
or you will start a second copy of it and the SDK will refuse the camera.

### What the flight does

| stage | what happens |
|---|---|
| `PREPARATION` … `TAKEOFF` | identical to the other nodes: health gates, Offboard, arm, ground wait, ramped climb |
| `HOLD` | `hold_seconds` at altitude, waiting for the flow to latch x/y. If the window is already in sight when the hold ends, it skips straight to `LOCK` |
| `SCAN` | the nose sweeps +45 deg, then -90, then +90, … about the takeoff heading at `yaw_rate`, until the window is confirmed |
| `LOCK` | yaw frozen at the heading the airframe actually has, x/y hold re-latched here, and it sits there |
| `LANDING` | starts `flight_seconds` after the **start of the climb**, whether or not a window was ever found |

The 40 s clock is checked before every stage handler, so a stage that gets
stuck cannot postpone the landing. The descent itself takes as long as it
takes on top of that.

A detection only stops the sweep if it is **live and sustained**:
`/window_detected` is already debounced in the detector, and `window_scan`
additionally requires it to have been true for `detect_seconds` and to be
no more than a second old. A camera that dies goes quiet, and quiet reads
as "keep looking" — never as a lock.

### Parameters

| parameter | default | what |
|---|---|---|
| `takeoff_altitude` | 1.0 | m above the arming point |
| `flight_seconds` | 40.0 | s from the start of the climb to the descent |
| `scan_span_deg` | 90.0 | total sweep width, centred on the takeoff heading |
| `yaw_rate` | 0.35 | rad/s (~20 deg/s) the yaw setpoint is walked at |
| `detect_seconds` | 0.4 | how long the detection must hold before the sweep stops |
| `relock_on_loss` | false | true = resume sweeping if the window is lost after the lock |
| `hold_seconds` | 5.0 | station keeping at altitude before the sweep |
| `image_topic` / `depth_topic` | see above | the ZED topics (`window_detect`) |
| `color` | green | which HSV range to look for: green, blue, red |
| `min_area` | 1500 | px^2 the contour must exceed |
| `show_windows` | false | cv2.imshow windows; needs a display |

Plus everything `offboard_sequence` takes for the climb and the descent
(`ground_wait_seconds`, `climb_speed`, `land_speed`, `min_altitude`,
`max_altitude`, `request_offboard_from_ros`).

### Status output

Same `/takeoff_status` topic and format. The detail field carries the sweep
and the clock: `scan37 22s` (37 deg of setpoint left in this leg, 22 s to
the landing), `lock-30 14s` (locked on a heading of -30 deg).

---

## 7. Optional: LCD status display

An Arduino running `arduino/tft_status/tft_status.ino` shows the stage, arm
state and altitude. It is started by default with the launch file:

```bash
ros2 launch drone_testing takeoff_test.launch.py lcd:=true lcd_port:=/dev/ttyACM0
```

Disable it with `lcd:=false`.

---

## 8. Optional: start at boot via systemd

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

## 9. Other nodes in the package

| node               | what it does                                                     |
|--------------------|------------------------------------------------------------------|
| `offboard_takeoff` | the autonomous takeoff / hold / land test (this README's subject) |
| `offboard_translate` | takeoff, then a 1 m horizontal move, then land (section 6)      |
| `offboard_sequence` | takeoff, then a list of moves / climbs / yaws, then land (section 6b) |
| `offboard_mission` | multi-waypoint offboard mission                                   |
| `zed_localization` | feeds ZED visual odometry into PX4 as `vehicle_visual_odometry`   |
| `lcd_status`       | drives the Arduino status display                                 |
| `pixhawk_node`     | MAVLink telemetry reader                                          |
| `cam`              | camera capture helper                                             |
| `window_detect`    | ZED window detection, publishes `/window_detected` (section 6c)    |
| `window_scan`      | takeoff, yaw sweep, lock onto the window, land after 40 s (section 6c) |

Other launch files:

- `translate_test.launch.py` — agent + `offboard_translate` (section 6)
- `sequence_test.launch.py` — agent + `offboard_sequence` (section 6b)
- `window_scan.launch.py` — agent + ZED + `window_detect` + `window_scan` (section 6c)
- `arm_test.launch.py` — agent + `offboard_mission`, for arm/disarm bench tests
- `offboard_launch.launch.py` — agent + ZED localization + `offboard_mission`

---

## 10. Troubleshooting

| symptom | cause / fix |
|---|---|
| `Waiting for VehicleStatus from PX4...` forever | DDS link down. Check the agent is running, the baud is 921600 both ends, `UXRCE_DDS_CFG` is set, and nothing else holds `/dev/ttyTHS1`. |
| `Not arming: rangefinder is NOT being fused (cs_rng_kin_consistent false)` | **Reboot the flight controller.** This flag is sticky: EKF2 only updates it while `in_air` is true (`range_height_control.cpp` runs the consistency check inside `if (_control_status.flags.in_air)`), so once it latches false in flight nothing on the ground can clear it. It comes back true at boot. See section 11.1. |
| `dist_bottom` stuck at exactly `EKF2_MIN_RNG` | The lidar is **not** healthy and EKF2 is synthesising the on-ground value: `_range_sensor.setRange(_params.ekf2_min_rng); setValidity(true)`. That number is not a measurement. `rng_ok=False` in the same log line confirms it. |
| `Not arming: need z_valid and dist_bottom_valid` | Rangefinder not being fused. Check the ARK Flow wiring and `EKF2_HGT_REF` / `EKF2_RNG_CTRL`. |
| `Offboard mode not entered in time` | PX4 rejected the mode. Usually pre-arm checks failing — look at the PX4 console or QGC for the reason. |
| `Arming rejected / timed out` | Pre-arm check failure, or the safety switch is not pressed. |
| `Altitude disagreement: ekf=... lidar=...` | The EKF datum and the rangefinder disagree by more than 32 cm. Normally an estimator reset mid-climb; the node correctly refuses to accept arrival. |
| `Offboard lost; PX4 has control now (nav_state AUTO_LAND(18))` | A PX4 failsafe fired. The node now logs `PX4 failsafe: ...` on the same line — read that. Offboard's *only* special mode requirement is `mode_req_offboard_signal`, so the usual culprit is `offboard_control_signal_lost` (a gap > `COM_OF_LOSS_T`, default 1.0 s, in the setpoint stream). See section 11.2. |
| `Offboard lost; PX4 has control now` | The TX switch moved, or PX4 failsafed. The node lets go on purpose. |
| `error: option --uninstall not recognized` on build | Stale `--symlink-install` state; see the build section above. |
| `q` / `k` do nothing | The node was started via `ros2 launch` or systemd, so stdin is not a tty. Run it with `ros2 run` in its own pane. |

---

### 10.1 The sticky rangefinder flag (`cs_rng_kin_consistent`)

This is the single most common reason the node refuses to arm, and it is
**not** a wiring fault — the sensor is usually fine.

EKF2 runs its rangefinder kinematic-consistency check only while airborne:

```c
// range_height_control.cpp
if (_control_status.flags.in_air) {
    _rng_consistency_check.update(...);
}
```

and `updateConsistency()` can only set the flag back to true when
`|vz| > 0.5 m/s`. So the flag starts `true` at boot, can only go false in
flight, and can only recover in flight. **On the ground it is frozen.** A run
that trips it poisons every subsequent run in that power cycle.

- **Fix:** reboot the flight controller (`reboot` in the nsh console, QGC's
  reboot button, or a power cycle). Then confirm before you touch anything:

  ```bash
  ros2 topic echo /fmu/out/estimator_status_flags --once | grep -E "cs_rng_hgt|cs_rng_kin_consistent"
  ```

  You want `cs_rng_hgt: true` **and** `cs_rng_kin_consistent: true`. If
  `cs_rng_hgt` is false and `cs_baro_hgt` is true, EKF2 has given up on the
  lidar and fallen back to the barometer — do not fly, the height datum is
  the baro and it drifts metres indoors.

- **Avoid:** reboot the FC at the start of every test session, and again after
  any flight where the flag tripped. Fly over flat, uniform floor — a mat, a
  cable, or a door threshold under the vehicle looks like vertical motion to
  the check and is a good way to trip it.

- **If it keeps tripping in flight:** the likely cause on a DroneCAN sensor
  like the ARK Flow is sensor lag. `EKF2_RNG_DELAY` (default 5 ms) is compared
  against the EKF's own `vz`; DroneCAN adds more latency than that, which
  produces a systematic innovation exactly when the vehicle is climbing or
  descending. Raise it (try 20–40 ms) and/or loosen `EKF2_RNG_K_GATE`.

Also worth knowing: `EKF2_MIN_RNG` is **not** a validity threshold. The
validity window comes from the sensor's own reported `min_distance` /
`max_distance` (0.02 m / 30 m on the ARK Flow). `EKF2_MIN_RNG` is the value
EKF2 *substitutes* when the lidar is unhealthy and the vehicle is at rest on
the ground — which is why a `dist_bottom` frozen at exactly that value means
"no measurement", not "9 cm".

### 10.2 Losing Offboard shortly after arming

Offboard has only three mode requirements in PX4 (`mode_requirements.cpp`):
angular velocity, attitude, and **offboard signal**. Nothing about position —
so a flaky position estimate cannot, by itself, kick you out of Offboard.
That narrows the causes a lot:

| flag in the new `PX4 failsafe:` log line | meaning | fix |
|---|---|---|
| `offboard_control_signal_lost` | No `OffboardControlMode` reached PX4 for `COM_OF_LOSS_T` (default **1.0 s**). Almost always a stall in the uXRCE-DDS uplink, not in the node. | Run the node with `ros2 run`, not inside a busy launch; cut the number of `/fmu/out` topics being bridged; check the agent with `-v6` for dropped uplink; consider `COM_OF_LOSS_T` 1.5–2.0. |
| `manual_control_signal_lost` | RC link lost while armed. | Keep the TX on. If you deliberately fly without RC, set `COM_RCL_EXCEPT` bit 2 (value `4`) to exempt Offboard. |
| `gcs_connection_lost` | QGC/datalink dropped, `COM_DL_LOSS_T` expired. | `COM_DLL_EXCEPT`, or keep QGC connected. |

Why it lands rather than holds: `COM_OBL_RC_ACT` defaults to **0 = Position
mode**, and this airframe has no usable horizontal position estimate on the
ground, so Position mode is unavailable and PX4 escalates down to Land —
`nav_state -> AUTO_LAND(18)`. Set `COM_OBL_RC_ACT = 4` (Land) so the
behaviour is at least explicit and predictable rather than the result of a
fallback chain.

---

## 11. Pre-flight checklist

1. Props **off** for the first run of any changed code.
2. **Reboot the flight controller.** `cs_rng_kin_consistent` is sticky across a
   whole power cycle and is the usual reason the node will not arm (10.1).
3. `ros2 topic echo /fmu/out/estimator_status_flags --once` → `cs_rng_hgt` and
   `cs_rng_kin_consistent` both true, `cs_baro_hgt` is *not* carrying the
   height on its own.
4. `ros2 topic echo /fmu/out/vehicle_local_position_v1 --once` → `z_valid` and
   `dist_bottom_valid` both true.
5. RC kill switch tested on the bench, this session.
6. `takeoff_altitude` set low (0.30 m).
7. Clear space around and above the vehicle — flow-only hold drifts.
8. You know which pane has the `q` key.
