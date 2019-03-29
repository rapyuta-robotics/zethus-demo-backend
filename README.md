# zethus-demo-backend
Publishers for demonstrating capabilities of zethus and amphion

## Install

### Build from Source

1. [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and the following build tools.

        sudo apt-get install python-wstool python-catkin-tools clang-format-3.8

1. Setup [Git-LFS](https://git-lfs.github.com/) if you have not already.
If git lfs is not setup, the mesh file will not be downloaded for visualizing `Mesh Resource`.
(note: Make sure to download the proper installation files from 'Downloads' section. The default download will always download the amd64 files)

1. Re-use or create a catkin workspace:

        export CATKIN_WS=~/ws_catkin
        mkdir -p $CATKIN_WS/src
        cd $CATKIN_WS/src

1. Download the required repositories and install any dependencies:

        git clone git@github.com:rapyuta-robotics/zethus-demo-backend.git
        cd zethus-demo-publisher
        git checkout publisher
        cd ..
        wstool init .
        wstool merge zethus-demo-backend/zethus-demo-backend.rosinstall
        wstool update
        rosdep install --from-paths . --ignore-src --rosdistro kinetic

(current working branch is `publisher`)

1. Configure and build the workspace:

        cd ..
        catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build

1. Source the workspace.

        source devel/setup.bash

## Run Docker

### Prerequisite

You must have a private rsa key `~/.ssh/id_rsa` that is not password protected and is attached to your Github and Bitbucket accounts. You must also have a working installation of `docker`.

1. Navigate to `$CATKIN_WS/src/zethus-demo-backend/.docker`. You should see the `Dockerfile` recipe in the directory.

1. Build the docker image

        cd $CATKIN_WS/src/zethus-demo-backend/.docker
        cp ~/.ssh/id_rsa id_rsa && docker build -t zethus-publisher:kinetic-source .; rm id_rsa

1. Run the docker image

    * Without the gui

            docker run -it --rm zethus-publisher:kinetic-source /bin/bash

## Run the publisher
```
roslaunch zethus_publisher publisher.launch start_webserver:=true webserver_port:=9090 pcl_port:=8888
```