# Changes and validation

## Build

The controller library now includes the coordinate-transform and hand-eye-calibration implementations. Both the library and executable depend on generated message targets. ROS header dependencies, Eigen, and jsoncpp are declared explicitly; unused PCL dependencies and the hardcoded x86 library directory were removed. The SDK is an imported library selected by target architecture or an explicit path. Runtime configuration and SDK files are installed with the node.

## Control sequence

The initial state advances once and publishes a detection request after a successful move. Empty queues finish before indexing. A completed placement removes its task and continues with the next target. Arm or tool-frame failures enter a terminal state instead of leaving a pending flag stuck. Hardware initialization failures cause a nonzero process exit. Tool IDs are checked before indexing, and tool frames 7 and 8 now match the bottle and bolt hand presets.

## Vision

The subscriber is owned by the controller. Detection messages accept variable object counts and reject inconsistent, non-finite, or reversed boxes. A complete validated snapshot selects explicitly configured fixed poses by label, only while the sequence is waiting. No pixel-to-3D conversion is inferred. The unused legacy point-cloud callback is not connected by this change.

## URDF

The copied `lefthand_10dof` package was replaced by the upstream repository pinned as a submodule. The description package uses the L20lite models identified as L10 by upstream documentation. Its loader handles escaped line endings and checks each mesh resource. Only the selected model family enters the catkin devel/install model directories, excluding unrelated upstream ROS packages.

## Test environment

Linux ARM64, Ubuntu 20.04, ROS Noetic, GCC 9, and the bundled ARM64 RealMan SDK. Build dependencies are listed in `tools/ros-noetic.Dockerfile`.

## Results

- ARM64 `catkin_make -j2 -DCMAKE_BUILD_TYPE=RelWithDebInfo`: passed, including message generation and final executable linkage.
- `catkin_make -j2 run_tests_HandTCPCommunication`: 76 distinct regression cases passed (11 grasp/vision, 11 TCP stream, 6 TCP recovery, 27 configuration, 16 calibration matrix, 5 launch parameters); no failures.
- `catkin_make -j2 install`: passed.
- Installed left and right URDF: each contains 21 links, 20 joints, and 42 mesh references; every referenced asset exists.
- Installed controller: all dynamic libraries resolve.
- Headless model launch using only the install package path: joint-state and TF messages received.

## Limits

No physical arm, hand, camera, or detector was used. Tests replace arm movement methods, while exercising the production state machine and a real ROS publisher/subscriber. Firmware compatibility, calibrated poses, collision clearance, firmware behavior during disconnect/reconnect, and physical grasp quality still require separate validation. The legacy hand-eye solver and point-cloud path are not validated as a complete vision pipeline. The existing uppercase ROS package name is preserved for topic/message compatibility.

## TCP follow-up

`TcpStream` owns a nonblocking socket and provides complete-frame reads and writes. A monotonic deadline remains fixed across partial transfers, EAGAIN, and EINTR. The controller shares one deadline across command write and response read, rejects incomplete responses, closes failed streams, and reconnects with a bounded attempt timeout and retry interval. SIGPIPE is suppressed per socket/write.

The wall timer and wall-rate main loop continue to service connections independently of ROS simulation time. The grasp state machine waits for valid initial feedback and enters its terminal state after a runtime transport failure. Restored communication does not resume that sequence. State publishers stay silent until fresh, complete feedback is available.

Regression tests use real loopback sockets and the production transport. A small send buffer and a delayed reader exercise short writes and backpressure. Fragmented, coalesced, truncated, delayed, and reset responses exercise receive handling. A full accept queue tests connection timeout without an external network dependency. ROS integration tests verify retry pacing, feedback invalidation, fresh data after reconnect, preservation of the last complete snapshot, and suppression of stale state publication.

TCP validation completed on the same ARM64 ROS Noetic environment: controller build/link, all 28 distinct regression cases, and installation passed. The installed executable has no unresolved dynamic-library dependencies. Recovery also passed with `/use_sim_time=true` and no clock messages. The tests use local sockets and a fake arm backend; physical device behavior was not exercised.

## Arm endpoint and pose-configuration follow-up

The arm IP, port, SDK socket timeout, speed percentage, and joint/tool JSON paths are private ROS parameters and launch arguments. An optional deployment YAML file loads limits and calibrated grasp targets into the node namespace. The RM65 model and six-axis control layout are unchanged.

The configuration parser validates parameter types and ranges, file schemas, all six joint components, all six tool-offset components, required tool keys, and fixed-target labels and IDs. JSON rejects duplicate keys and trailing data. Numeric values must be finite and fit the SDK float range; nonzero values that would underflow to zero are rejected. Optional joint limits and Cartesian workspace bounds are checked during loading and before motion commands. Joint targets retain their legacy six-angle layout; Cartesian SDK poses now receive a quaternion consistent with their Euler angles.

Configuration is parsed into a candidate and committed only on success. Invalid configuration stops startup before hand or arm connection. A failed reload disables further arm motion commands and clears active targets. SDK calls are isolated behind overridable methods so tests verify endpoint/timeout/speed forwarding and confirm that invalid input produces no initialization, connection, or movement calls.

Validation passed on Linux ARM64, Ubuntu 20.04, ROS Noetic, and GCC 9: controller build/link, all 53 distinct test cases, and installation. The installed executable has no unresolved dynamic-library dependencies. Devel and install launch checks confirmed default JSON paths, endpoint/speed overrides, and deployment-YAML precedence; the automated launch suite also verifies namespaced loading. No physical arm or hand was used.

## Hand-eye calibration follow-up

`hand_eye_calibration_file` is a private parameter and launch argument, with the installed package file as its default. The strict loader reads exactly 16 row-major values into a candidate matrix and never indexes beyond its fixed size. It rejects malformed or non-finite values, trailing data, an invalid homogeneous row, non-orthonormal or reflected rotations, and translation outside the SDK float range. Rotation and determinant tolerances are `1e-5`; the homogeneous-row tolerance is `1e-9`. The bundled matrix passes without modification.

Configuration loading validates the calibration before hardware connections and commits it with the other settings. Constructors no longer perform implicit reads. Missing or invalid calibration never falls back to identity, and zero translation does not erase a valid rotation. Failed reloads invalidate getters and prevent further arm commands. The legacy class and general helper use the same strict reader.

Regression coverage includes short, oversized, malformed, non-finite, and trailing-data files; row-major ordering; rigid-transform validity and tolerances; zero-translation rotations; controller/legacy-reader behavior; and launch/YAML path selection. No physical calibration, camera, hand, or arm was used.

Validation passed on Linux ARM64 / ROS Noetic: build/link, all 76 distinct regression cases, installation, installed-library resolution, and devel/install calibration-path overrides through launch arguments and deployment YAML. All 16 matrix-reader tests also passed with AddressSanitizer and UndefinedBehaviorSanitizer, including oversized input, with no sanitizer findings.

The sanitizer check was run inside the Linux test container from the repository root:

```sh
g++ -std=c++11 -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Ifixedgrasp/src/HandTCPCommunication/include -I/usr/include/eigen3 \
  fixedgrasp/src/HandTCPCommunication/test/calibration_matrix_regression.cpp \
  fixedgrasp/src/HandTCPCommunication/src/CalibrationMatrix.cpp \
  -lgtest -lpthread -o /tmp/calibration-sanitized
ASAN_OPTIONS=detect_leaks=1 /tmp/calibration-sanitized
```
