# Base image with ROS 2 Jazzy Desktop
FROM osrf/ros:jazzy-desktop-full

# ── Build-time environment ────────────────────────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# ── System dependencies ───────────────────────────────────────────────────────
# FIX: added ros-jazzy-tf-transformations — required by turtlebot3_drive.py and
#      tb3_follow_tb1.py (tf_transformations Python package).
# FIX: added ros-jazzy-nav2-bt-navigator so follow_dynamic_point.xml BT is
#      available inside the container.
# FIX: added ros-jazzy-behaviortree-cpp for BT runtime support.
# FIX: added python3-transforms3d as fallback quaternion/euler helper.
# FIX: consolidated into a single RUN layer for smaller image size.
RUN apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    curl \
    ros-jazzy-rmw-implementation \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-tf-transformations \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-behaviortree-cpp \
    python3-transforms3d \
    mesa-vulkan-drivers

# ── CycloneDDS config ─────────────────────────────────────────────────────────
RUN printf '<CycloneDDS>\n\
  <Discovery>\n\
    <ParticipantIndex>auto</ParticipantIndex>\n\
    <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>\n\
  </Discovery>\n\
</CycloneDDS>\n' > /etc/cyclonedds.xml

ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# ── TurtleBot3 model env (overridable at run time via -e) ─────────────────────
# FIX: TURTLEBOT3_MODEL was missing — launch files read this to pick params/model.
ENV TURTLEBOT3_MODEL=burger

# ── Gazebo resource path ──────────────────────────────────────────────────────
# FIX: GZ_SIM_RESOURCE_PATH was missing — Gazebo cannot find TB3 models without it.
ENV GZ_SIM_RESOURCE_PATH=/opt/ros2_ws/install/tb3_multi_robot/share/tb3_multi_robot/models

# ── rosdep ────────────────────────────────────────────────────────────────────
RUN rosdep update

# ── Workspace ─────────────────────────────────────────────────────────────────
WORKDIR /opt/ros2_ws
RUN mkdir -p src && chmod -R a+rwx /opt/ros2_ws

# FIX: original cloned upstream arshadlab/tb3_multi_robot; changed to your fork
#      so that follow_tb3.launch.py, tb3_follow_tb1.py, and
#      burger_nav2_params_tb3.yaml are present in the image.
RUN git clone https://github.com/gglaspell/tb3_multi_robot.git src/tb3_multi_robot -b follow

# ── Install ROS deps and build ─────────────────────────────────────────────────
RUN /bin/bash -c "\
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install"

# ── Permissions ───────────────────────────────────────────────────────────────
RUN chmod -R a+rX /opt/ros2_ws

# ── Shell conveniences ────────────────────────────────────────────────────────
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash"   >> ~/.bashrc \
 && echo "source /opt/ros2_ws/install/setup.bash"      >> ~/.bashrc \
 && echo "export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}" >> ~/.bashrc

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /opt/ros2_ws
