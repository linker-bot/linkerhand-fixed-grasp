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
- `catkin_make -j2 run_tests_HandTCPCommunication`: 11 regression cases passed; no failures.
- `catkin_make -j2 install`: passed.
- Installed left and right URDF: each contains 21 links, 20 joints, and 42 mesh references; every referenced asset exists.
- Installed controller: all dynamic libraries resolve.
- Headless model launch using only the install package path: joint-state and TF messages received.

## Limits

No physical arm, hand, camera, or detector was used. Tests replace arm movement methods, while exercising the production state machine and a real ROS publisher/subscriber. Firmware compatibility, calibrated poses, collision clearance, TCP framing and disconnect recovery, and physical grasp quality still require separate validation. The legacy hand-eye solver and point-cloud path are not validated as a complete vision pipeline. The existing uppercase ROS package name is preserved for topic/message compatibility.
