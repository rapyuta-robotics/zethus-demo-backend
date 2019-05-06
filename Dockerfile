# zethus-publisher:kinetic-source
# Copies the source code, installs the remaining debian dependencies, and
# builds the package

FROM ros:kinetic

# Remove warning 'Could not determine the width of the terminal.'
ENV TERM xterm

ENV CATKIN_WS=/root/ws_catkin
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Install SHH
# Note: --fix-missing is run to fix an issue with an outdated ros-kinetic-rviz-visual-tools
RUN apt-get -qq update --fix-missing && \
    apt-get -qq install -y wget ssh python-catkin-tools python-wstool curl

WORKDIR $CATKIN_WS/src

# Copy source
COPY . /root/ws_catkin/src/
RUN wstool init . && \
    wstool merge zethus-demo-backend.rosinstall && \
    wstool update


# Note that because we're building on top of kinetic-ci, there should not be
# any deps installed unless something has changed in the source code since the
# other container was made (they are triggered together so should only be
# one-build out of sync)
RUN rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false

# Configures environment
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8

# Status rate is limited so that just enough info is shown to keep Docker from
# timing out, but not too much such that the Docker log gets too long (another
# form of timeout)
RUN catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --jobs 1 --limit-status-rate 0.001 --no-notify

CMD ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && source $CATKIN_WS/devel/setup.bash && roslaunch zethus_publisher publisher.launch start_webserver:=true webserver_port:=9090 pcl_port:=8888"]