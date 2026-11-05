FROM ros:noetic-ros-base-focal
RUN apt-get update && apt-get install -y --no-install-recommends build-essential pkg-config libeigen3-dev libjsoncpp-dev ros-noetic-serial ros-noetic-tf ros-noetic-tf2-geometry-msgs ros-noetic-rostest ros-noetic-urdf ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher && rm -rf /var/lib/apt/lists/*
