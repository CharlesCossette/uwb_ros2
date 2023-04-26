FROM osrf/ros:humble-desktop

ARG USERNAME=ros
ARG WORKSPACE=/home/$USERNAME/workspace/src
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME 
RUN useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME 

# [Optional] Add sudo support for the non-root user
RUN apt-get update 
RUN apt-get install -y sudo git-core bash-completion 
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME
RUN chmod 0440 /etc/sudoers.d/$USERNAME 

# Cleanup
RUN rm -rf /var/lib/apt/lists/* 
RUN echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# We are still the root user at this point. install pip
RUN apt-get update 
RUN apt-get install -y python3-pip
RUN pip3 install --upgrade pip

# ROS2 yells a warning with a version of setuptools that is too high.
# So intentially use an olderversion
RUN pip3 install setuptools==58.2.0

# Now switch to non-root user "ros"
USER $USERNAME
RUN mkdir -p $WORKSPACE
WORKDIR $WORKSPACE
