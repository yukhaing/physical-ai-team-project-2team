FROM robotis/open-manipulator:5.0.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    python3-vcstool \
    ros-jazzy-cv-bridge \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-usb-cam \
    && rm -rf /var/lib/apt/lists/*

ARG CYCLO_CONTROL_COMMIT=ceffbd7562028f6b317e462911e2a0991b9ba735
RUN cd /root/ros2_ws/src && \
    git clone https://github.com/ROBOTIS-GIT/cyclo_control.git && \
    cd cyclo_control && \
    git checkout "${CYCLO_CONTROL_COMMIT}" && \
    vcs import /root/ros2_ws/src < cyclo_control_ci.repos

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths /root/ros2_ws/src --ignore-src -r -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/* && \
    bash -c 'source /opt/ros/jazzy/setup.bash && \
      source /root/ros2_ws/install/setup.bash && \
      cd /root/ros2_ws && \
      colcon build --symlink-install --packages-up-to cyclo_motion_controller_ros \
        --cmake-args -DCMAKE_BUILD_TYPE=Release'

RUN mkdir -p /root/omx_box_project_ws && \
    printf '%s\n' \
      'if [ -f /root/omx_box_project_ws/install/setup.bash ]; then' \
      '  source /root/omx_box_project_ws/install/setup.bash' \
      'fi' >> /root/.bashrc && \
    printf '%s\n' \
      "alias omx_box_build='cd /root/omx_box_project_ws && colcon build --symlink-install'" \
      "alias omx_box_source='source /root/omx_box_project_ws/install/setup.bash'" \
      >> /root/.bashrc

WORKDIR /root/omx_box_project_ws
CMD ["bash"]
