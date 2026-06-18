FROM ros:noetic-ros-base

# --- CARLA Python API download FIRST, in its own cached layer, so later
#     changes to apt/pip steps never force a re-download of this large file ---
WORKDIR /opt/carla
RUN apt-get update && apt-get install -y wget && rm -rf /var/lib/apt/lists/*
RUN wget https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.15.tar.gz -O carla.tar.gz \
    && tar -xzf carla.tar.gz PythonAPI \
    && rm -f carla.tar.gz \
    && ls PythonAPI/carla/dist

ENV PYTHONPATH="/opt/carla/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg:/opt/carla/PythonAPI/carla"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-rosdep \
    python3-catkin-tools \
    python-is-python3 \
    git \
    unzip \
    ros-noetic-cv-bridge \
    ros-noetic-derived-object-msgs \
    ros-noetic-tf \
    ros-noetic-ackermann-msgs \
    ros-noetic-rviz \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy opencv-python simple-pid transforms3d networkx pygame

# --- CARLA ROS bridge (built from source for guaranteed x86_64 + ROS Noetic compatibility) ---
WORKDIR /catkin_ws/src
RUN git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git carla_ros_bridge && \
    rm -rf carla_ros_bridge/rqt_carla_control \
           carla_ros_bridge/rviz_carla_plugin \
           carla_ros_bridge/carla_manual_control \
           carla_ros_bridge/pcl_recorder

WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true"

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# --- Our ADAS pipeline (copied in at build time; also volume-mounted for live edits) ---
COPY adas_pipeline /catkin_ws/src/adas_pipeline
COPY adas_msgs /catkin_ws/src/adas_msgs

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

WORKDIR /catkin_ws
CMD ["bash"]
