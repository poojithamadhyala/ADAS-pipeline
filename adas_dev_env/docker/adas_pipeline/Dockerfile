FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-derived-object-msgs \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy opencv-python

WORKDIR /catkin_ws

CMD ["bash"]
