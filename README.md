# zethus-demo-backend
Publishers for demonstrating capabilities of zethus and amphion

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

## Run
```
roslaunch zethus_publisher publish_markers.launch start_webserver:=true webserver_port:=9090
```