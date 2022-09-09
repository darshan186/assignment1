 #! /bin/bash
 
USER_NAME=tii_dev
DOCKER_REPO="vivekrk44/tii_interview"
CONTAINER_NAME="tii_dev"
WORKSPACE_DIR=~/${CONTAINER_NAME}_docker
CMD=""

if [ "$1" != "" ]; then
    CONTAINER_NAME=$1
    WORKSPACE_DIR=~/$1_docker
    echo "Container name:$1 WORSPACE DIR:$WORKSPACE_DIR" 
fi

if [ ! -d $WORKSPACE_DIR ]; then
    mkdir -p $WORKSPACE_DIR
fi

if [ "$2" != "" ]; then
    CMD="-c \"$2\""
    echo "$CMD will be passed to the container ..."
fi

DOCKER_OPTS=

XAUTH=/tmp/.docker.xauth
xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]
then
    echo XAUTH file does not exist. Creating one...
    touch $XAUTH
    chmod a+r $XAUTH
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    fi
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]
then
  echo "[$XAUTH] was not properly created. Exiting..."
  exit 1
fi

PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )


echo "WORKSPACE_DIR: $WORKSPACE_DIR";
echo "Username:" $USER_NAME
#not-recommended - T.T please fix me, check this: http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:root
 
echo "Starting Container: ${CONTAINER_NAME} with REPO: $DOCKER_REPO"
 
if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    docker rm ${CONTAINER_NAME}
fi
    docker run -it \
        --gpus all \
        --user=$USER_NAME \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --network host \
        --privileged \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="$WORKSPACE_DIR:/home/$USER_NAME:rw" \
        --volume="/dev:/dev:rw" \
        --volume="/etc/localtime:/etc/localtime:ro" \
        --volume="/dev/input:/dev/input" \
        --volume="$XAUTH:$XAUTH" \
        --env="XAUTHORITY=$XAUTH" \
        --workdir="/home/$USER_NAME" \
        --name=${CONTAINER_NAME} \
        $DOCKER_OPTS \
        ${DOCKER_REPO} \
        bash $CMD
xhost -local:root
