# Base image with ROS 2 Jazzy Desktop
FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

# ── Build-time environment ────────────────────────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# ── System dependencies ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-rmw-cyclonedds-cpp \
    mesa-vulkan-drivers \
    && rm -rf /var/lib/apt/lists/*

ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# ── TurtleBot3 model env (overridable at run time via -e) ─────────────────────
# FIX: TURTLEBOT3_MODEL was missing — launch files read this to pick params/model.
ENV TURTLEBOT3_MODEL=burger

# ── Gazebo resource path ──────────────────────────────────────────────────────
# FIX: GZ_SIM_RESOURCE_PATH was missing — Gazebo cannot find TB3 models without it.
ENV GZ_SIM_RESOURCE_PATH=/opt/ros2_ws/install/tb3_multi_robot/share/tb3_multi_robot/models

# ── Workspace ─────────────────────────────────────────────────────────────────
WORKDIR /opt/ros2_ws
RUN mkdir -p src && chmod -R a+rwx /opt/ros2_ws

# Resolve dependencies from this checkout's manifest. Copying the manifest
# separately keeps this expensive layer cached while source files are edited.
COPY package.xml src/tb3_multi_robot/package.xml
RUN apt-get update \
    && rosdep update \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
        --skip-keys ament_python \
        -r -y \
    && rm -rf /var/lib/apt/lists/*

# ── CycloneDDS config ─────────────────────────────────────────────────────────
RUN printf '<CycloneDDS>\n\
  <Domain>\n\
    <Discovery>\n\
      <ParticipantIndex>auto</ParticipantIndex>\n\
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>\n\
    </Discovery>\n\
  </Domain>\n\
</CycloneDDS>\n' > /etc/cyclonedds.xml

# Build the source that is actually present in this workspace rather than
# cloning the remote branch again during every image build.
COPY . src/tb3_multi_robot

# ── Build ──────────────────────────────────────────────────────────────────────
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --packages-select tb3_multi_robot

# ── Permissions ───────────────────────────────────────────────────────────────
RUN chmod -R a+rX /opt/ros2_ws

# ── Shell conveniences ────────────────────────────────────────────────────────
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash"   >> ~/.bashrc \
 && echo "source /opt/ros2_ws/install/setup.bash"      >> ~/.bashrc \
 && echo "export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}" >> ~/.bashrc

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
WORKDIR /opt/ros2_ws
