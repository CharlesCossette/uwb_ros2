# uwb_ros2

## Getting started
To start developing, you need to install VSCode along with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Then, press CTRL + SHIFT + P to open command pallet and pick _Dev Containers: Open Folder in Container..._

Choose this folder, and select the existing Dockerfile in this folder if prompted. VSCode will reload and you will now be inside the container,
with ROS installed, the workspace build, and the node ready to run.

Connect a UWB module and run, open a terminal in VSCode

    ros2 run uwb_driver uwb_node

## TODOs

1. Docker setup so that a single command starts the node? Hopefully can reduce to just cloning the repo and running a docker command or two
2. How to log data once the container is launched?
3. Lets say someone shows up with another container for another sensor, how to connect them so we can record everything with one command?
4. Implement common list scheduler. Have to change the way the sequence is specified....
5. Test the full setup here.