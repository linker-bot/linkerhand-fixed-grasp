# LinkerHand Fixed Grasp

ROS 1 fixed-position grasp demonstration for a RealMan arm and a LinkerHand dexterous hand. The controller moves between configured arm poses, applies object-specific hand presets, and places objects on one of two trays.

An external detector can select calibrated, fixed grasp targets by label. Bounding boxes do not provide depth or generate 3D grasp poses. This repository does not include a camera driver, object detector, general grasp planner, or grasp-success estimator.

## Requirements

- Linux with ROS Noetic, catkin, a C++11 compiler, Eigen3, and jsoncpp.
- ROS packages declared in `fixedgrasp/src/HandTCPCommunication/package.xml`.
- A compatible RealMan SDK. The bundled `libRM_Service.so` is Linux ARM64. Other architectures require a matching library through `-DRM_SERVICE_LIBRARY=/absolute/path/libRM_Service.so`.
- Compatible hand firmware and robot tool frames. Check the configured hand freedom, speed, effort, addresses, tool frames, and all motion targets against the connected equipment.

## Checkout and build

```sh
git clone --recurse-submodules https://github.com/linker-bot/linkerhand-fixed-grasp.git
cd linkerhand-fixed-grasp
# For an existing checkout:
git submodule update --init --recursive
source /opt/ros/noetic/setup.bash
rosdep install --from-paths fixedgrasp/src --ignore-src -r -y
cd fixedgrasp
catkin_make -j2 -DCMAKE_BUILD_TYPE=RelWithDebInfo
source devel/setup.bash
```

The URDF submodule is under `vendor/linkerhand-urdf`, outside the catkin source tree. Its upstream README maps L10 to the `L20lite` model directory. `linkerhand_description` loads those models, normalizes the upstream escaped line endings, and resolves mesh paths without modifying the submodule.

## View the model

```sh
roslaunch linkerhand_description display.launch side:=left
# Headless model and TF publication:
roslaunch linkerhand_description display.launch side:=right rviz:=false
```

This launch provides model visualization. The hardware controller's `JointState` topics carry protocol values and are not mapped to the upstream URDF joint names or units.

## Configure and run the controller

`target_pose.json` must contain exactly five six-axis joint targets: initial position, tray B approach, tray B placement, tray A approach, and tray A placement. These are joint angles, stored in the existing `position.{x,y,z}` and `orientation.{x,y,z}` fields; they are not Cartesian poses or quaternions. All six components are required numeric values. The unused legacy `orientation.w` is optional, but must be numeric if supplied.

`tools_Tcp.json` retains the original tool-offset data. The controller switches to tool frames already configured on the arm; it does not create or calibrate those frames from this file. Each tool entry must contain six numeric `Tcp` components. The eight required tool keys are `Silk`, `Eggs`, `Battery`, `pills`, `Ball`, `Screwdriver`, `bottle`, and `Bolt`; they are loaded in preset-ID order rather than JSON key order. Preset IDs are 1 silk, 2 eggs, 3 battery, 4 pills, 5 ball, 6 screwdriver, 7 bottle, and 8 bolt.

Configure `~fixed_grasp_targets` as an array of objects with `label`, `id` (1–8), and `pose` (six finite values: x, y, z in metres and rx, ry, rz in radians). The pose must be calibrated in the arm's active work frame for the selected tool frame. A detected label selects that fixed pose, regardless of the box's pixel position. Labels and preset IDs must each be unique within the configuration. No grasp targets are provided by default.

```sh
roslaunch HandTCPCommunication HandCtrl_node.launch
```

The existing launch defaults use hand IP `192.168.1.40`, protocol port 20008, and freedom mode 16. The default arm endpoint is `192.168.1.18:8080`, supplied through ROS parameters. Starting this node connects to hardware and begins the initial movement. Load the calibrated target parameters under `/arm` before starting it, or add a `rosparam` file inside the launch node.

The arm and motion configuration parameters are private to the controller node (normally `/arm`):

- `arm_ip`: numeric IPv4 address; default `192.168.1.18`.
- `arm_port`: integer 1–65535; default 8080.
- `arm_timeout_ms`: SDK socket timeout, integer 1–60000; default 5000.
- `arm_speed_percent`: joint and Cartesian movement speed, integer 1–100; default 20.
- `hand_eye_calibration_file`: absolute hand-eye matrix path; defaults to the package `CalibrateMatrix_EyeInHand.txt`.
- `target_pose_file` and `tools_tcp_file`: absolute JSON paths. Defaults resolve to the package files in both devel and install spaces.
- `joint_limits_deg`: optional six `[min, max]` pairs for joints 1–6, in degrees.
- `workspace_bounds_m`: optional three `[min, max]` pairs for fixed grasp x, y, z, in metres.

Bounds are inclusive and must contain finite numeric values in ascending order. No physical joint limits or workspace bounds are assumed when those parameters are absent. Configure them for the actual robot, work frame, and tool frames. Structural and numeric checks do not establish collision clearance or reachability. This controller retains the six-axis RM65 SDK mode; changing the endpoint does not select another arm model.

For example, override the endpoint and speed without changing source files:

```sh
roslaunch HandTCPCommunication HandCtrl_node.launch arm_ip:=192.168.1.20 arm_port:=8080 arm_timeout_ms:=3000 arm_speed_percent:=15 target_pose_file:=/absolute/path/target_pose.json tools_tcp_file:=/absolute/path/tools_Tcp.json
```

Use `motion_config_file:=/absolute/path/deployment.yaml` to load a private ROS parameter YAML file containing the endpoint settings, limits, and calibrated `fixed_grasp_targets`. It is loaded after the launch parameters, so values in that file take precedence over arguments with the same names.

Malformed JSON, duplicate keys, trailing data, missing components, incorrect types, non-finite values, float overflow/underflow, invalid preset IDs, and targets outside configured bounds cause startup to fail before either hardware connection. The parser commits a configuration only after every section passes. A failed reload disables further arm motion commands and clears active targets; it does not leave a partially loaded batch. Joint and Cartesian commands are also checked immediately before SDK calls. The legacy `pose_X`/`pose_RX` launch parameters belong to the unused `show_left()` path; the active sequence uses the JSON joint targets and `fixed_grasp_targets`.

At the initial position, the controller publishes `detect_objects` on `/aimo/robot_control/OrderDetect0bj` and waits for an `OrderDetectResult` on `/aimo/robot_ai/orderDetectobj_result`:

- `labels`: zero or more object labels.
- `bboxes`: four finite coordinates per label, ordered x1, y1, x2, y2, with non-reversed bounds.

One valid snapshot creates a batch. Unknown labels are ignored; duplicate preset IDs are queued once. Empty snapshots finish the batch when fixed targets are configured. Without configured targets, the controller remains waiting. Each queued target receives one grasp-and-place sequence. Motion or tool-frame errors stop further sequence commands until restart. Completion means commands finished, not that a successful physical grasp was measured.

## Legacy visual point-cloud callback

`GetVisualData_subCallback()` accepts the legacy custom `PointCloud2` snapshot format. The default launch still subscribes to the 2D detector interface; it does not subscribe this point-cloud callback to a topic.

A nonempty snapshot requires the fields `cls`, `x1`, `y1`, `z1`, `x2`, `y2`, `z2`, `a`, `b`, `c`, and `d`. Each required field must contain exactly one `FLOAT32`. `cls` is an integer preset ID from 1 to 8; IDs must be distinct within a snapshot. `x1/y1/z1` are camera-frame coordinates and `x2/y2/z2` form a nonzero camera-frame direction vector. `a/b/c/d` are preserved metadata and do not determine the grasp pose. All required values must be finite, including when `is_dense` is false.

Fields must have distinct names and nonoverlapping byte ranges within `point_step`. Valid extra fields and point padding are accepted. The decoder respects `row_step`, including row padding in organized clouds, handles both byte orders, and accepts unaligned offsets. The data length must equal `row_step * height`, and each row must fit its declared points. At most eight points are supported. An empty snapshot has zero width, height 0 or 1, zero row stride, and no data; it may omit all fields with zero point stride.

The callback accepts snapshots only with validated configuration while waiting for a result. Parsing and task construction finish before either the decoded objects or task queue is replaced. Invalid snapshots leave both unchanged and do not mark detection complete. Valid empty snapshots clear the batch; objects absent from a new snapshot are removed, while attempt counts for retained IDs are preserved.

Task position uses the existing observation pose and validated hand-eye matrix. Direction is rotated into the base frame without translation; its base-XY projection determines yaw in `[-π/2, π/2]`. A zero or near-vertical direction without a stable XY projection is rejected. Roll and pitch come from the transformed camera orientation using the existing XYZ Euler convention. Every task member is initialized, and transformed positions are checked against the SDK float range and any configured workspace bounds.

The callback retains the legacy fixed observation pose. It does not obtain a synchronized live arm pose or establish a complete camera-driven grasp pipeline. Sensor schema, frame convention, and physical grasp behavior still require device validation.

## Hand-eye calibration file

Set `hand_eye_calibration_file:=/absolute/path/calibration.txt` on the launch command, or set the same private parameter in the deployment YAML. The default resolves to `CalibrateMatrix_EyeInHand.txt` in the package in both devel and install spaces. Paths are not prefixed with the package directory when overridden.

The file contains exactly 16 whitespace-separated numeric values in row-major order for a 4×4 homogeneous transform. The existing one-value-per-line layout and a four-row layout are both accepted, including blank lines, CRLF, and scientific notation. Headers, comments, trailing text, extra values, partial numbers, NaN, and infinity are rejected.

The loader checks the homogeneous last row against `[0, 0, 0, 1]` with an absolute tolerance of `1e-9`. It checks each component of `RᵀR − I` and `det(R) − 1` against `1e-5`, accommodating the rounded rotation in the bundled file. Translation must also fit the SDK float range. Reflections, scaling, shear, and singular rotations fail validation. Values are not normalized, rescaled, inverted, or replaced with an identity matrix. A zero translation is valid, including when the rotation is not identity.

The matrix is loaded as part of the complete controller configuration before hardware connections. Failed reads or validation do not commit a partial configuration. A failed reload invalidates the controller's calibration and disables subsequent arm commands; the getter reports an error instead of returning stale data. `HandEyeCalibration` construction no longer reads a file: standalone callers must use `LoadEyeInHand()` or `ReadCalibrationResultXFile_EyeInHand()` before requesting a result. The legacy `CalibrationEyetohandXRead_path()` helper also uses the strict loader and now expects an absolute path.

These checks establish a valid rigid-transform representation, not a correct physical calibration. The caller must supply the appropriate frame direction and consistent translation units. The existing eye-in-hand path multiplies arm pose × calibration × camera/object pose; its metre-based inputs require matching calibration units. Camera calibration quality and the full legacy vision pipeline remain unverified.

## TCP transport and recovery

The hand connection uses fixed-size binary frames, retaining the existing wire layout. Nonblocking reads and writes continue until the entire frame is transferred. One monotonic deadline covers both the command and its response; partial progress or interrupted system calls do not reset that deadline. EOF, reset, timeout, or another I/O failure closes the connection. Partial responses never update or publish hand feedback.

The launch exposes these private parameters:

- `hand_port`: 20008 by default, range 1–65535.
- `hand_connect_timeout_ms`: 250 ms per connection attempt.
- `hand_io_timeout_ms`: 100 ms for a complete request/response exchange.
- `hand_reconnect_delay_ms`: 1000 ms between failed attempts or after a lost connection.

Timeout and retry parameters accept 1–60000 ms. The retry delay starts after an attempt ends. A wall-clock timer and main-loop rate schedule communication, so paused ROS simulation time does not suspend recovery. A callback can spend up to the connection budget plus the exchange budget when reconnecting; this is bounded synchronous I/O, not a real-time control guarantee.

Startup connection failure still exits with a nonzero status. A runtime failure stops the grasp sequence and suppresses hand/glove state publication. The connection retries automatically and resumes feedback publication only after a complete response. The next exchange sends the latest command state, including any externally updated setpoints, rather than an unfinished byte suffix. The grasp sequence requires a node restart to run again.

The protocol has no transaction identifier or acknowledgement of physical execution. A lost response cannot establish whether the preceding command took effect; reconnection is not an exactly-once command guarantee. Reconnection does not stop an arm movement already executing inside the separate RealMan SDK.

## Validation

```sh
cd fixedgrasp
catkin_make -j2 run_tests_HandTCPCommunication
catkin_test_results build/test_results
catkin_make -j2 install
```

The regression suite uses the production state machine and ROS subscriber with a fake arm backend; it does not connect to hardware. It covers initial-state transitions, empty and exhausted queues, batch processing, motion and tool failures, invalid detection data, fixed-pose selection, subscriber lifetime, and target configuration. Additional TCP tests cover fragmented and coalesced frames, write backpressure, connect/read/write deadlines, interrupted waits, EOF/reset, retry delay, partial-response rejection, and feedback publication after recovery. Configuration tests cover malformed files and parameters, bounds, atomic loads, failure before hardware connections, and endpoint/speed forwarding to a fake SDK backend. Calibration tests cover file bounds, malformed values, rigid-transform checks, zero-translation rotations, parameter overrides, and invalidation after a failed reload. Point-cloud tests cover malformed layouts and fields, byte order, row padding, sparse IDs, initialized tasks, and atomic callback updates.

`tools/ros-noetic.Dockerfile` provides an ARM64-compatible build environment using the bundled SDK. See [validation notes](docs/validation.md) for the checks performed and remaining limitations.

## License

The repository license is Apache-2.0. Upstream URDF assets and bundled SDK files retain their own notices and licensing terms.
