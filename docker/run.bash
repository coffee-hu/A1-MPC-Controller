IMG="$1"
#IMG=$(basename $1)
# Truncate last "/" in case using bash-complete for directory name
if [[ $IMG == */ ]]; then
  IMG=${IMG::-1}
fi
echo $IMG
current_dir=$(pwd)
xhost + local:
  
docker run -d \
    --network host \
    --cap-add=IPC_LOCK --cap-add=sys_nice \
    -v "$current_dir/..:/root/A1_ctrl_ws/src/A1_Ctrl" \
    --device /dev/input \
    --name ${IMG//[:\/]/_} \
    --privileged \
    --restart=always \
    $IMG
    
xhost - local: