# uwb_ros2

All commands should be run in the directory of this repo.

## Quick start
First, build the docker image 

    docker build -t uwb_ros .

Then, start a container using the above image, while mounting our host `/dev` directory to `/dev` inside the container for USB access. The default command runs the UWB node

    docker run -it -v /dev:/dev uwb_ros

## Development
To start developing, you need to install VSCode along with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

Then, press CTRL + SHIFT + P to open command pallet and pick _Dev Containers: Open Folder in Container..._

Choose this folder, and select the existing Dockerfile in this folder if prompted. VSCode will reload and you will now be inside the container,
with ROS installed, the workspace built, and the node ready to run.

Connect a UWB module and run, open a terminal in VSCode

    ros2 run uwb_driver uwb_node

### More "manual" development setup without VS Code
The development docker image is slightly different from the running/production docker image. We use a multi-stage build so that both images are specified using a single Dockerfile. In this section, we'll explore how to start a terminal in the development image without using VS Code (mainly for insight). To build a development image, 

    docker build -t uwb_ros_dev --target development .

Now we must start a container but also do a few things while doing so:
- bind mount this source directory to the workspace in the container
- mount the local /dev/ directory to /dev/ in the container (for USB device access)

```bash
docker run -it --mount type=bind,src="$(pwd)",target=/home/ros/workspace/src/uwb_ros2/ -v /dev:/dev uwb_ros_dev /bin/bash
```
Now in any editor, you can modify the node's source code and it will be reflected instantly in the container. In the terminal that originated from the previous command, install dependences and run the node with
```bash
cd ~/workspace/src/uwb_ros2/uwb_driver 
pip3 install .
cd ~/workspace
colcon build --symlink-install
source ./install/setup.bash
ros2 run uwb_driver uwb_node
```




## TODOs

1. How to log data once the container is launched?
2. Lets say someone shows up with another container for another sensor, how to connect them so we can record everything with one command?
3. Implement common list scheduler. Have to change the way the sequence is specified....
4. Test the full setup here.