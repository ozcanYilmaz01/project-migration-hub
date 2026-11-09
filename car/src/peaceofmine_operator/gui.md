# Operator GUI

Browser dashboard for driving a PeaceOfMine SVEA. The Xbox on your laptop
does **not** publish a ROS topic. Chrome reads the pad and sends
`{linear_x, angular_z, deadman}` over a WebSocket. ROS starts on the car
in `operator_web`, which bridges those messages onto ROS. The separate
`operator_control` node authorizes commands and publishes `/<ns>/cmd_vel`.

Yes, this is meant to run on the **real car**. Simulation and hardware share
that same `cmd_vel` path. Hardware replaces `sim_svea` with MAVROS → PX4.
The physical RC transmitter remains the override.

## Dashboard layout

Choose drive input above the camera. Click the camera to capture WASD; Escape
releases it. The task workbench groups Search, Confirm arc, Probe scan and
Mark ground with one live status and inspection progress. Probe controls open
for confirmed targets and active probe cycles, showing actual depth and whether
the probe is descending or retracting. Configure maximum depth and contact
voltage there; use Home / travel setup for calibration.

Diagnostics below the workspace holds filtered detector/angle plots, controller,
probe, map, power and detailed link telemetry. Mission log reads the shared record on the car and exports it as CSV. Refreshing
a browser releases its control lease and stops active motion. Completed arc
confirmation stays available to the next owner while the vehicle remains in place.
Driving or starting a new search invalidates the previous probe target.

Both cameras are optional with the normal launch command. An enabled camera
retries independently when disconnected. During a
probe scan the dashboard promotes the probe feed only when it has frames.
ADC graphs show the filtered voltage used by the car. Calibration can also show unfiltered voltage. Setup retains input, camera and hardware calibration.

## Field map and GNSS

Signal & map opens at street level on the first fresh receiver position. The dot follows the car and the circle shows its estimated horizontal error in metres. Drag to stop following and select **Follow** to resume. The trail and logged ground marks remain visible when the fix becomes stale. The local grid is available in the map selector. OpenStreetMap supplies the base tiles over the browser's internet connection. The car supplies all positions, trails and marks independently of those tiles.

The field recorder reads the Pi GPIO UART at `/dev/ttyAMA0`, 115200 baud. Set `gnss_device` and `gnss_baud` in the operator launch for another receiver. NAV-PVT supplies coordinates and horizontal accuracy together. GGA-only output provides a position with unknown accuracy. Outdoor RTK localization supplies the ROS `/fix` topic when enabled. Simulation skips the UART.

**Setup → GNSS** shows the same live position, accuracy circle, trail and ground marks. Each map has its own zoom and Follow control. The control owner can set the acceptable horizontal error (default 1 m) and maximum wait (default 5 s, set 0 for immediate capture). A ground mark commands the pump immediately and logs the event on the car. With a fresh fix within target, the recorder attaches it immediately. Otherwise it keeps the best available fix during the wait, ending early on vehicle movement or loss of control telemetry. Outside the target, the log shows an uncertain position and its estimated error. With no fix it records the shot with its position unavailable. Physical paint or water delivery requires visual confirmation.

All clients read the same mission entries and recent trail (up to 900 points). The recorder uses `field.sqlite3` beside the resolved operator config, or `field-simulation.sqlite3` in simulation. The standard container mounts this directory from the host. Override `field_log_path` to choose another persistent location. Clearing the mission log removes its recorded events and map marks. The driven trail remains.

In **Setup → Marking**, the owner can hold the manual test in either RC drive
mode with PX4 armed. ROS commands the pump on **AUX4 / misc0 / CH5**.
This car uses duty mask 32 and channel 6 parameter endpoints 0 and 4096.
The idle setting defaults to 0%. In direct RC mode the transmitter selects the
physical output. Setup displays the command percentage sent by ROS.

## Architecture

```
Xbox on the laptop
  → Chrome Gamepad API          (no ROS on the laptop)
  → WebSocket drive messages
  → operator_web                (on the SVEA)
  → /self/operator/command      std_msgs/String
  → operator_control            (lease, safety, drive watchdog)
  → /self/cmd_vel               geometry_msgs/Twist
  → twist_consumer
  → /self/mavros/manual_control/send
  → PX4
```

`cmd_vel` is published as soon as a valid drive command arrives (browser
WebSocket or `/joy`), not on the next watchdog tick. Publishing continues
only while you hold the control lease, the physical RC permits ROS driving,
and commands are newer than 0.25 s. No additional GUI arming or Shift key is
required. After that the
control node sends zeros for 0.5 s and goes silent. Stick mapping is linear
with a 0.12 deadzone; there is no extra command smoothing.

Optional second input: plug the Xbox into the **SVEA USB** and pass
`use_joy:=true`. Then `joy_node` publishes `/self/joy` and the gateway
drives from that. A live `/joy` stream wins over browser sticks. You do
not need this if the pad stays on the laptop.

## Will the laptop pad work?

Yes, if the dashboard is opened as a **secure context**:

| How you open the GUI | Laptop Xbox |
| --- | --- |
| `https://<svea-ip>:<port>` | Works (accept the self-signed cert) |
| `http://localhost:8080` on the SVEA itself | Works only if the pad is on the SVEA |
| `http://<svea-ip>:8080` from the laptop | **No** — Chrome hides the Gamepad API |

## Safety before hardware

- Stop any other operator / sim launch first (two stacks must not share
  `cmd_vel` or port 8080).
- Keep the vehicle lifted or in a clear area for the first test.
- Have the physical RC ready. Its kill switch stops actuation. Releasing the
  drive input, losing browser focus, or losing the control connection stops
  software driving.
- Do not enable localization, lidar, or RTK until those sensors are
  brought up separately.

## Exact steps (real car, Xbox on the laptop)

Run these on the SVEA. Privileged Docker is required so `/dev` and the
PX4 serial device are visible (`util/run`, not `DEV=1`).

### 1. One container, this workspace

```bash
util/run
```

Inside the container, from `/svea_ws`:

```bash
colcon build --symlink-install --packages-up-to peaceofmine_operator
source /opt/ros/jazzy/setup.bash
source /opt/svea/jazzy/setup.bash
source /svea_ws/install/setup.bash
```

If that fails with `can't copy '.../usb_camera_auto.py': doesn't exist or not a regular file`, the build tree still lists a script that is no longer in the package. Wipe that package and rebuild, then source again in the **same** shell (do not launch on a failed overlay):

```bash
rm -rf build/peaceofmine_operator install/peaceofmine_operator
colcon build --symlink-install --packages-select peaceofmine_operator
source /opt/ros/jazzy/setup.bash
source /opt/svea/jazzy/setup.bash
source /svea_ws/install/setup.bash
```

Confirm the PX4 is present:

```bash
ls /dev/serial/by-id/usb-SVEA_PX4_AUTOPILOT_0-if00
```

### 2. Launch hardware (not sim)

Still inside the container, after sourcing:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=false
```

This starts MAVROS/LLI, `twist_consumer`, and the gateway. XML is the main
operator configuration. Without certificates the dashboard uses HTTP on port
8080. Choose a free port explicitly, for example `port:=8082`, if it is busy.

It does **not** start `sim_svea`. Payload simulation and `joy_node` stay
off unless you pass `simulate_payload:=true` or `use_joy:=true`.

For a remote browser controller, provide HTTPS certificates. To generate a
development certificate, replace the example address with the SVEA's actual IP:

```bash
mkdir -p /tmp/operator-tls
openssl req -x509 -newkey rsa:2048 -nodes \
  -keyout /tmp/operator-tls/key.pem -out /tmp/operator-tls/cert.pem \
  -days 30 -subj '/CN=svea-mine' \
  -addext 'subjectAltName=DNS:localhost,DNS:svea-mine,IP:127.0.0.1,IP:192.168.1.100'
```

Then launch with both certificate paths:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=false \
  tls_cert:=/tmp/operator-tls/cert.pem \
  tls_key:=/tmp/operator-tls/key.pem
```

If the Wi-Fi address changes, regenerate the certificate with the new address.

### 3. On the laptop

1. Plug in the Xbox 360 and press a button.
2. Open `https://<svea-ip>:8080` (same IP as in the certificate; use your chosen port).
3. Accept the certificate warning.
4. Confirm the input toolbar shows the pad; its live graphic is under Diagnostics.
5. **Take control** and select ROS authority on the physical RC.
6. Use the RC to permit motion; there is no separate dashboard arm/disarm step. RT forward, LT reverse, left stick steer. Release the input deadman to stop driving. The header STOP releases dashboard control; click **Take control** before a new operation. Settings → Stop / release torque stops only the selected actuator and keeps dashboard control. Opening Setup releases keyboard capture. Gamepads, wheels and RC through ROS continue to drive during ordinary Setup. An active actuator calibration pauses ROS driving until calibration ends.

The status line must read `ROS cmd_vel …`. Stick lights alone are not
enough. The virtual forward camera is first-person, so the car will not
slide across that view; watch the vehicle and the speed readout.

### 4. Stop

Stop with the physical RC, then `Ctrl+C` the launch.

## Simulation only

Hardware-free check on the same `cmd_vel` path:

```bash
ros2 launch peaceofmine_operator operator.launch.xml is_sim:=true
```

Open `http://localhost:8080` on the machine that has the pad, or use
HTTPS for a remote laptop as above.

## Settings

Settings has separate Arm (default ID 1) and Probe (default ID 2) panels on the
shared ArbotiX. Switching requires stopped motion. Reconnect while stopped to
rescan after connecting a missing servo. Arm sweep calibration retains its
three points, saved with **Save recorded limits** in `operator_config.json`.

To calibrate **maximum extension**, start with the probe at the physical top when the encoder connects or let Confirm arc home it. Then jog down to the desired maximum extension, release, enter the measured travel
from the top in millimetres, then choose **Save current position as maximum
extension**. The saved distance and corresponding encoder span provide the
millimetre conversion and travel limit for the main Probe slider.

Shared robot settings and hardware calibration live in **`src/peaceofmine_operator/operator_config.json`**:
`arm` contains the servo ID and minimum/centre/maximum encoder positions;
`probe` contains the servo ID, measured maximum extension and encoder span;
`metal_detector` contains zero ADC, trigger ADC and reference voltage;
`arm_motion` contains maximum speed and acceleration;
`adc` (and a separate `adc_simulation` section) contains conversion settings, plot routes and contact trigger;
`cameras` contains shared source/rotation defaults and robot capture settings.
Optional sections are created when first saved from Settings.
This regular JSON file belongs in Git. Settings saves update it on disk; commit
those changes to record new measurements in Git history.

The nodes locate the installed package file independently of the launch working
directory. With the recommended `colcon build --symlink-install`, saves resolve
the installed symlink and update this source file in the mounted repository.
A non-symlink installation uses its installed copy instead. For an explicit
alternative, pass `config_file:=/absolute/path/operator_config.json`.
Saved sections take precedence over legacy calibration launch parameters;
those parameters are fallbacks only when a section is absent.
Updates are atomic and locked across processes, preserving the other sections.
The first valid probe encoder position after a controller restart is assumed to be the top. Confirm arc and Probe scan establish a new zero by retracting the connected probe to its top stop. A brief driver reconnect retains the session reference.

Close Settings, take control, and use the main Probe slider. Hardware targets
now reach the MX-64 driver; simulation retains its fake payload. The main
control uses the saved travel and the session zero. It uses the saved down or retract speed and holds torque at the target. The main
Chart.js graph shows absolute servo load (0–100%), not calibrated pressure or
force; missing or stale load displays as unavailable. The hold remains subject
to the same 60-second command timeout. **Stop probe**, control-owner
loss, stale driver telemetry, PX4 permission loss, rover movement, or a
60-second movement timeout stops the command stream. The driver and firmware
retain their independent command/permission timeouts. Stop arm sweeping before
moving the probe: the controller grants motion to one selected servo at a time.

The launch `arm_speed` is in servo register units, not degrees/s. Values above
the firmware limit of 80 are capped with a warning rather than disconnecting
the adapter. Calibration setup errors retain the detected ID list.

One failed servo-bus read reported as gateway status 32 is retried once, without
retrying motion writes or renewing permission. A repeated failure still stops
and disconnects the driver. Settings -> Servos -> Reconnect explicitly rescans
with targets cleared; it does not resume a sweep or jogging. Session calibration
is retained for the same role/ID. Fresh RC/owner permission and a new movement
request are required after reconnection.

| Controller mapping | When to use |
| --- | --- |
| Auto | Default. Browser `standard` layout if the pad reports it, otherwise Linux Xbox 360 (triggers on axes 2 and 5). |
| Standard | Force LT/RT on buttons 6/7. |
| Xbox 360 (Linux) | Force bipolar trigger axes. |

Choose WASD above the camera, then click the forward camera to capture keys.
Escape or clicking elsewhere releases them. Hold Space to sweep; release to stop.
Set the forward and reverse keyboard throttle limits in Setup → Controls. The car saves both percentages.

The physical RC is authoritative. RC override blocks browser driving but permits
the control owner to sweep the servo while PX4 is active and RC kill is clear.
RC loss, kill, stale heartbeat, gateway permission timeout, or control-owner
disconnect stops the arm. Settings jogging does not require saved limits;
sweeping requires minimum, center and maximum calibration. The input toolbar shows RC channels in physical override and when **RC through ROS** is selected. In the ROS switch position, the latter sends drive commands while fresh transmitter samples and owner presence continue. The default channels are steering CH1 and throttle CH2; adjust `rc_steering_channel` and `rc_throttle_channel` in launch XML
to match your transmitter. PWM visualization assumes 1000/1500/2000 endpoints.

In Setup → Arm, record the physical Left, Centre and Right positions, then choose **Save limits**. The endpoints define the numeric minimum and maximum in either recording order. Centre sets the zero for the detector angle. An outer probe position beyond a saved endpoint is clipped to that endpoint, and the scan map identifies the limiting side.

Settings calibration jog/position controls use the full EEPROM joint range,
not the saved sweep endpoints. Check mechanical clearance before extending the
calibration range. Normal sweeps still enforce saved minimum/maximum bounds.
The Servos tab also exposes maximum sweep speed and acceleration while stopped.
The main sweep slider spans that configured speed range with no separate 28
degrees/s clamp. Speed register values 1..1023 are supported; actual achievable
speed depends on load, acceleration and travel distance. Motion settings apply
for this driver session; the launch XML export includes speed and acceleration
for persistence across restarts.

The arm driver requires [safe_arm firmware v1](../../DevTools/servodemo/firmware/safe_arm/README.md).
It sends a sweep profile once and refreshes permission every 50 ms while the
ROS safety gate allows motion. The new firmware uses fixed endpoint goals with a bounded speed ramp and
100 Hz feedback target. It confirms low measured speed before reversing,
without a fixed endpoint pause. This requires flashing the updated firmware; restarting ROS alone does
not install it. Sweep speed changes adjust the bounded ramp while running. The slider range
uses the configured arm maximum, including its ROS status message. The default
116 motor units correspond to 79.344 degrees/s; this is a configurable software
limit, not the motor's physical maximum. Acceleration and available travel can
prevent reaching the requested speed. If permission heartbeats stop for 350 ms,
it cancels motion and releases torque. RC kill still uses ROS/USB; no extra
wiring is required. The firmware must be flashed separately before this driver
can move the arm. See the linked build and commissioning instructions.

Arm position is published on each 50 ms control tick, independent of the slower
electrical diagnostics. Dashboard telemetry targets 20 Hz; the radar redraws
at the display refresh rate (60 Hz on a 60 Hz display). Charts reuse their
layout between telemetry updates for smooth scrolling. The virtual drive
view, map pose, radar and probe graphic interpolate measured samples with
50 ms display delay; they never predict through telemetry gaps. Raw values,
commands and safety state remain unmodified. Input graphics follow browser
frames, and measurements retain their original sampling rate. Camera frame rate depends on the camera stream.
These are scheduling targets, not guaranteed measured rates.
Confirmed firmware motion stops retain the connection after torque-off is
verified and require a new user motion request, not adapter rediscovery.
Transport disconnects retry discovery in the background after a two-second
backoff. Reconnection restores the selected role and session calibration but
never resumes an old sweep or jog. RC kill keeps a healthy connection open.

Settings → Metal detector has editable **Baseline (V)** and **Mine level (V)**
fields. **Apply & save** stores both on the vehicle, including fractional values.
The filtered detector voltage and both saved levels share the same graph. You can
enter levels without fresh readings; **Measure baseline** needs recent samples and
updates only the baseline. The filter never estimates or changes these levels.

The Detector tab gives the signal and arm-angle chart most of the dock. **Expand chart** uses the workspace, and **Restore chart** or Escape returns to the normal layout. The voltage axis scales using up to two minutes of detector readings. Hover a sample to read its filtered voltage and fixture angle. A minimum span of 1% of the input range keeps small fluctuations readable in context. Thresholds and calibration lines appear when they fall within the measured range. In Setup, turn off **Auto scale** to view the full detector voltage range.

Setup → Metal detector shows filtered input voltage. Measure averages recent
filtered samples for the baseline. Save writes the baseline and mine level to the car.
In Mission → Sweep and alert, enter an alert voltage. The alert follows the direction
from baseline toward the mine level, including detectors whose voltage falls over metal.
Older percentage-based alert settings display their equivalent voltage until you edit the limit.

Setup → Sensor board shows filtered voltage for each enabled channel. Select the unfiltered voltage comparison when tuning the filter.
The hardware comparator limits are entered in volts at the selected input range.
ADC CSV exports report voltages in V.

## Files

- `dashboard/index.html`, `app.js`, `style.css` — pad graphic, mapping, HUD
- `scripts/operator_web_node.py` — website, WebSocket and ROS transport
- `scripts/operator_control_node.py` — control lease, RC authority, drive timeout
- `scripts/servo_driver_node.py` — shared serial bus and hardware safety
- `scripts/arm_controller_node.py`, `probe_controller_node.py` — separate actuator behavior
- `scripts/detector_node.py` — sensor interpretation and plot history
- `architecture.md` — full ownership map and audit findings
- `launch/operator.launch.xml` — hardware or simulation, selected by `is_sim`
- `launch/operator_sim.launch.py` — `sim_svea` + simulated payload
- `gui_plan.md` — longer roadmap (cameras, map, payload, mission record)

### Detector smoothing

Setup → Metal detector shows the applied detector voltage and a dashed preview when you edit smoothing.
Save smoothing applies the same time constant to the car’s ADC channels, including probe contact.
Setup → Sensor board → Live readings exposes the same setting and previews every enabled input.
The default is 100 ms. Zero disables smoothing. Larger values suppress more noise and delay the response.
Raw counts, volts and scaled ROS topics remain available. Each input is filtered
independently; `adc/filtered_samples` carries its filtered volts and scaled
value with the original raw count and timestamp.

### Confirm and probe scan

Confirm arc homes the connected probe before moving the arm. With the probe disconnected, detector confirmation can still run. Probe scan homes once before the first position and waits for the return stroke between positions.

Search sweeping defaults to 36°/s. Use the main actions in order: Search, Confirm arc, Probe scan. Confirmation
uses the even pass count saved under Setup → Metal detector (2 by default) at the selected search speed. It compares a broad arc against each pass's
local noise and compares their measured half-height footprints. A majority must share at least 50% overlap and include both sweep directions.
The result averages their forward and return median centers. The probe scan uses the point count and spacing from Setup → Arm (default 7 points at 10° over ±30°), clipped to the saved arm limits and corrected for the detector/probe
mounting offset saved under Setup → Arm (default 30°). To calibrate from an existing scan, subtract the arm angle that physically centered the probe from the confirmed detector bearing. Each new scan uses the saved offset for all its points. The scan map labels the confirmed bearing and the commanded arm center. Each completed marker shows measured contact depth or the configured target depth reached. A dash indicates an unavailable depth. Contact or maximum depth ends each descent; the probe retracts
before the next angle. In Setup → Probe, **Down speed** and **Retract speed** set the two stroke speeds in mm/s (both default to 5). Automatic homing uses the retract speed once travel is calibrated. **Automatic home load** sets the motor load that establishes zero at the top (10% by default). Stop inspection cancels the sequence while retaining the
dashboard control lease. Global STOP and RC/session loss cancel outstanding goals.

The Probe panel exposes maximum depth and the saved Above/Below contact limit in
volts. This changes the configured probe input's threshold, not its raw samples.
During descent the selected filtered ADC input and its configured threshold determine contact. An input already active at the start of a stroke records contact at the current depth. Setup → Probe contact shows the applied state and the voltage at which contact clears. Manual controls and technical motor
readings remain available under their details sections.

### Probe direction

Settings → Probe → **Reverse probe direction** is saved on the vehicle. It reverses
extend/retract jogging, homing and depth moves together. After changing direction, Confirm arc can establish a fresh top reference before manual depth moves. The shaft-position slider always uses raw encoder
position. Simulation stores its direction separately from the car.

### Probe contact feedback

Probe contact polling targets 20 Hz; voltage and temperature refresh at 1 Hz.
Actual rate depends on serial response time.
Below Manual probe, the live chart plots filtered contact sensor voltage and the saved contact threshold in volts, alongside measured encoder depth in millimetres.
The motor diagnostics chart autoscales load (%) and current (mA) independently,
expanding immediately and shrinking gradually. Values retain their units.
Traces use light smoothing; the two-second peak uses unfiltered samples.
Hold clear of the ground and select **Zero contact** to capture the recent
holding baseline. It stays fixed through subsequent contact and movement.
The reference is session-only and clears on disconnect or actuator change.
Position error is shown in shaft degrees. Stale contact telemetry shows no
reading; zeroing requires fresh holding samples and control ownership.

## ADS1115 sensor and probe contact trigger

Both operator launches start `ads1115_node.py` in the vehicle namespace. The node
owns I2C acquisition and sends configuration changes to the settings service.
The probe controller evaluates contact using the filtered samples. The
register driver, acquisition worker, ROS node, gateway telemetry cache, and browser
settings live in separate files. The gateway never opens the I2C bus.

Acquisition starts automatically with the node and continues without a browser
or control lease. Saved settings and signal assignments are restored before the
bus opens. Hardware/configuration errors are reported; no dummy readings replace
them. Stop the standalone ADS1115 devtool before launching the operator.
To enable inputs, open **Setup → Sensor board**, take control and select **Save changes**. Simulation uses labelled
demo samples and a separate `adc_simulation` section in the shared
`operator_config.json`; hardware uses the `adc` section. Override `config_file`
in either launch with an absolute mounted path to choose a different shared file.
The earlier `.operator/ads1115.json` and `ads1115-sim.json` files are imported once
when the matching section is absent. `adc_settings_path` is retained only as an
optional legacy import path; all new saves go to the shared configuration.

Select the detector input in **Setup → Metal detector** and the contact input in
**Setup → Probe contact**. Both sensors may share a channel. The default detector
input is A2 and the default contact input is A0. **Setup → Sensor board** shows
the assignments beside the input enable, multiplier and offset controls. It also
provides register settings, live readings, filter preview and file import/export.

In **Probe contact**, watch the filtered voltage with the sensor clear and then
pressed. Choose the input, direction, voltage limit, hysteresis and debounce,
enable detection and select **Save contact settings**. The graph previews the
draft limit against the selected channel. The applied status shows the car's contact decision, the voltage that activates it, the release voltage, and the debounce time. The mission screen uses that same decision. Hysteresis must leave a reachable release voltage. If older settings contain an unreachable hysteresis, the car applies 0 V and shows a notice until you save a correction.

Each descent requires a fresh contact reading. If contact is already active, the car records the measured starting depth and voltage without extending. A probe below its home position retracts. Contact during descent also ends the downstroke and starts retraction.

Relative ROS topics (under `/self/` with the default vehicle name):

| Topic | Type | Contents |
| --- | --- | --- |
| `adc/state` | `std_msgs/String` JSON | 20 Hz batches containing every acquired sample (`seq`, epoch `time`, `mux`, signed `raw`, `volts`, `scaled`, `clipped`), node session, revision, configuration, plot routes, readback, and errors. The web bridge adds the probe-owned contact result. |
| `adc/{input}/raw` | `std_msgs/Int16` | Latest signed raw reading per input per publication tick. |
| `adc/{input}/volts` | `std_msgs/Float64` | Latest voltage per input per publication tick. |
| `adc/{input}/scaled` | `std_msgs/Float64` | Latest scaled value per input per publication tick. |
| `adc/command` | `std_msgs/String` JSON | `{request_id, action: "settings", settings: {config, routes, probe_trigger}}`. Acquisition is always on; start/stop commands are rejected. |
| `adc/command_result` | `std_msgs/String` JSON | Matching `request_id`, `ok`, and error `message` when rejected. A successful settings result means saved; `applied == revision` confirms device readback. |
| `adc/samples` | `peaceofmine_interfaces/AdcSamples` | Timestamped batches for robot consumers; includes configuration revision and validity. |
| `probe/contact` | `peaceofmine_interfaces/ContactState` | Probe-owned contact result. Invalid means unknown. Consumers must also enforce a receipt timeout. |

Input topic names are `a0`, `a1`, `a2`, `a3`, `a0_a1`, `a0_a3`, `a1_a3`, and
`a2_a3`. Scalar topics publish only when new samples exist; use `adc/samples` for
all samples, timestamps, freshness, and sequence-gap detection. Node acquisition
continues without browsers. Browser history is bounded to 60 seconds / 30,000
samples and is not a persistent logger. Browser setting changes use the operator
control lease; ROS command access follows the deployment's ROS access controls.

Install the updated workspace requirements (`smbus2`) or rebuild the Docker image
before hardware use, then rebuild `peaceofmine_operator`. The launching account
needs I2C device access. The node opens the configured bus automatically at startup.


## ROS responsibilities and persistent configuration

See [architecture](architecture.md) for ownership, safety and persistence, and
[ROS graph](ros_graph.md) for the current nodes, topics, services and actions.

Both launch commands now run the same arm and probe controllers. In simulation,
`servo_driver` uses an in-memory device/plant, and ADC acquisition uses generated
samples. No physical device is opened. Probe simulation uses the same startup zero and automatic inspection homing as hardware.

`servo_driver` owns all serial I/O and enforces bounded speed, acceleration,
exclusive actuator ownership, operation identities and watchdogs. Controllers
exchange typed commands and telemetry without synchronous register requests.
The website is served by `operator_web`, which communicates through ROS.

A missing gateway heartbeat stops actuation after 0.5 seconds. The lease survives a brief stall and expires after 3 seconds; closing the socket or taking over releases it immediately. RC
permission, a control lease and held drive input are required; there is no separate
browser arming latch. Drive commands expire after 0.25 seconds.
If servo commands stop arriving for 250 ms, the driver releases torque. During an automatic positioning or probe action, the same controller can resume with a fresh response to a new driver challenge within 750 ms of its last renewal, while permission and feedback are still live. An expired action requires a new request.

All runtime saves go through `operator_settings` into `operator_config.json`.
Schema/document/section revisions protect concurrent changes. The old
`calibration_file` launch argument and migration from a sibling `calibration.json`
remain supported. Saved and applied revisions are separate; ADC readback confirms
application, while actuator configuration changes stop movement and require a new
start. Live arming, leases, home position and acquisition-running state are not saved.

Persistent settings include ADC configuration/scaling, plot assignments, probe
contact thresholds, arm/probe calibration, motion limits, detector calibration and
robot camera defaults. Simulation ADC and arm/probe settings use separate sections; simulated calibration cannot overwrite hardware calibration. Camera
capture changes apply after restart. Laptop camera IDs and browser controller
mappings remain local to each browser. Ports, TLS and device-enable flags remain
launch options.

Probe motion and detector calibration do not require odometry. Fresh RC permission,
the actuator connection and its own calibration/fault checks determine readiness.
Sweeping, alignment and marking do not require probe setup, but wait for an active
probe or marking operation to finish. A detector location marker still requires
actual odometry.

The browser depth slider holds the requested depth until Stop, safety loss or the
60-second limit. ROS `probe/execute` target actions instead finish and release torque
at depth. During a probe cycle, ADC contact records the hit depth and immediately
starts retraction; the scan advances only after that return completes. Manual depth
moves stop extending on contact. Completion requires the applied goal, matching
operation and settled speed within the command tolerance, capped to one quarter of
short strokes. A timeout reports the phase and remaining distance instead of
claiming a completed return.
Homing uses the separate motor-load threshold, initially 10%. Verify the physical
top after homing.

`operator_gateway.py` remains a compatibility entry point for the website server;
`arm_servo.launch.xml` starts the driver and both controllers. The obsolete combined
servo executable does not open a competing serial connection.
