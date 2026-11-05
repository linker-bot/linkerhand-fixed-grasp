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

`target_pose.json` contains five six-axis joint targets: initial position, tray B approach, tray B placement, tray A approach, and tray A placement. These are joint angles, stored in the existing `position.{x,y,z}` and `orientation.{x,y,z}` fields; they are not Cartesian poses or quaternions.

`tools_Tcp.json` retains the original tool-offset data. The controller switches to tool frames already configured on the arm; it does not create or calibrate those frames from this file. Preset IDs are 1 silk, 2 eggs, 3 battery, 4 pills, 5 ball, 6 screwdriver, 7 bottle, and 8 bolt.

Configure `~fixed_grasp_targets` as an array of objects with `label`, `id` (1–8), and `pose` (six finite values: x, y, z in metres and rx, ry, rz in radians). The pose must be calibrated in the arm's active work frame for the selected tool frame. A detected label selects that fixed pose, regardless of the box's pixel position. No grasp targets are provided by default.

```sh
roslaunch HandTCPCommunication HandCtrl_node.launch
```

The existing launch defaults use hand IP `192.168.1.40`, protocol port 20008, and freedom mode 16. The arm endpoint remains `192.168.1.18:8080` in `InitArm()`. Starting this node connects to hardware and begins the initial movement. Load the calibrated target parameters under `/arm` before starting it, or add a `rosparam` file inside the launch node.

At the initial position, the controller publishes `detect_objects` on `/aimo/robot_control/OrderDetect0bj` and waits for an `OrderDetectResult` on `/aimo/robot_ai/orderDetectobj_result`:

- `labels`: zero or more object labels.
- `bboxes`: four finite coordinates per label, ordered x1, y1, x2, y2, with non-reversed bounds.

One valid snapshot creates a batch. Unknown labels are ignored; duplicate preset IDs are queued once. Empty snapshots finish the batch when fixed targets are configured. Without configured targets, the controller remains waiting. Each queued target receives one grasp-and-place sequence. Motion or tool-frame errors stop further sequence commands until restart. Completion means commands finished, not that a successful physical grasp was measured.

## Validation

```sh
cd fixedgrasp
catkin_make -j2 run_tests_HandTCPCommunication
catkin_test_results build/test_results
catkin_make -j2 install
```

The regression suite uses the production state machine and ROS subscriber with a fake arm backend; it does not connect to hardware. It covers initial-state transitions, empty and exhausted queues, batch processing, motion and tool failures, invalid detection data, fixed-pose selection, subscriber lifetime, and target configuration.

`tools/ros-noetic.Dockerfile` provides an ARM64-compatible build environment using the bundled SDK. See [validation notes](docs/validation.md) for the checks performed and remaining limitations.

## License

The repository license is Apache-2.0. Upstream URDF assets and bundled SDK files retain their own notices and licensing terms.
