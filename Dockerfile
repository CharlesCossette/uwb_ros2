FROM osrf/ros:humble-desktop AS development

ARG USERNAME=ros
ARG WORKSPACE=/home/$USERNAME/workspace/src
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME 

# [Optional] Add sudo support for the non-root user
RUN apt-get update \
    && apt-get install -y sudo git-core bash-completion \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

# We are still the root user at this point. install pip.
# ROS2 yells a warning with a version of setuptools that is too high.
# So intentially use an older version
RUN apt-get update \
    && apt-get install -y python3-pip\
    && pip3 install --upgrade pip\
    && pip3 install setuptools==58.2.0


# Cleanup
RUN rm -rf /var/lib/apt/lists/* 
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# # Now switch to non-root user "ros"
USER $USERNAME
RUN mkdir -p $WORKSPACE
WORKDIR $WORKSPACE

FROM development as run
USER $USERNAME
RUN mkdir uwb_ros2
COPY --chown=$USERNAME:$USERNAME . /home/$USERNAME/workspace/src/uwb_ros2/
RUN cd /home/$USERNAME/workspace/src/uwb_ros2/uwb_driver \
    && pip3 install . \
    && cd /home/$USERNAME/workspace \
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build --symlink-install

CMD ["bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source ~/workspace/install/setup.bash && ros2 run uwb_driver uwb_node"]