XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        sudo echo $xauth_list | sudo xauth -f $XAUTH nmerge -
    else
        sudo touch $XAUTH
    fi
    sudo chmod a+r $XAUTH
fi

if [[ -x "$(command -v dpkg)" ]] && dpkg --compare-versions "$(docker version --format '{{.Server.Version}}')" gt "19.3"; then
  DOCKER_OPTS="--gpus=all"
else
  DOCKER_OPTS="--runtime=nvidia"
fi

xhost + local:

sudo docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    -v /home/huzhanyong/A1-QP-MPC-Controller:/root/A1_ctrl_ws/src/A1_Ctrl \
    $DOCKER_OPTS \
    --privileged \
    --network host \
    --ipc=host \
    --restart=always \
    --name a1_unitree_gazebo_docker \
    a1_unitree_gazebo_image

xhost - local: