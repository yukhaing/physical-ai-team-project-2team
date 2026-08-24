FROM robotis/open-manipulator:5.0.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    python3-numpy \
    python3-scipy \
    python3-vcstool \
    tmux \
    ros-jazzy-cv-bridge \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-usb-cam \
    && rm -rf /var/lib/apt/lists/*

ARG CYCLO_CONTROL_COMMIT=ceffbd7562028f6b317e462911e2a0991b9ba735
RUN cd /root/ros2_ws/src && \
    git clone https://github.com/ROBOTIS-GIT/cyclo_control.git && \
    cd cyclo_control && \
    git checkout "${CYCLO_CONTROL_COMMIT}" && \
    vcs import /root/ros2_ws/src < cyclo_control_ci.repos

COPY patches/cyclo_movej_s_curve.patch /tmp/cyclo_movej_s_curve.patch
COPY patches/cyclo_movej_s_curve_feedback_hold.patch /tmp/cyclo_movej_s_curve_feedback_hold.patch
COPY patches/cyclo_movej_s_curve_bounded_settle.patch /tmp/cyclo_movej_s_curve_bounded_settle.patch
COPY patches/cyclo_movej_s_curve_terminal_trim.patch /tmp/cyclo_movej_s_curve_terminal_trim.patch
COPY patches/cyclo_movej_idle_wait.patch /tmp/cyclo_movej_idle_wait.patch
COPY patches/cyclo_movej_idle_quiet.patch /tmp/cyclo_movej_idle_quiet.patch
RUN cd /root/ros2_ws/src/cyclo_control && \
    git apply /tmp/cyclo_movej_s_curve.patch && \
    git apply /tmp/cyclo_movej_s_curve_feedback_hold.patch && \
    git apply /tmp/cyclo_movej_s_curve_bounded_settle.patch && \
    git apply /tmp/cyclo_movej_s_curve_terminal_trim.patch
RUN cd /root/ros2_ws/src/cyclo_control && \
    git apply /tmp/cyclo_movej_idle_wait.patch
RUN cd /root/ros2_ws/src/cyclo_control && \
    git apply /tmp/cyclo_movej_idle_quiet.patch

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
      'if [ -f /root/ros2_ws/install/setup.bash ]; then' \
      '  source /root/ros2_ws/install/setup.bash' \
      'fi' \
      'if [ -f /root/omx_box_project_ws/install/setup.bash ]; then' \
      '  source /root/omx_box_project_ws/install/setup.bash' \
      'fi' >> /root/.bashrc && \
    printf '%s\n' \
      "alias omx_box_build='cd /root/omx_box_project_ws && colcon build --symlink-install'" \
      "alias omx_box_source='source /root/omx_box_project_ws/install/setup.bash'" \
      >> /root/.bashrc

RUN mkdir -p /root/.rviz2 && \
    ln -sfn /root/omx_box_project_ws/install/omx_box_control/share/omx_box_control/rviz/omx_box_project.rviz \
      /root/.rviz2/default.rviz

WORKDIR /root/omx_box_project_ws
CMD ["bash"]
