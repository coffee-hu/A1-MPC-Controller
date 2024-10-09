#!/usr/bin/env bash

IMG="$1"

# Truncate last "/" in case using bash-complete for directory name
if [[ $IMG == */ ]]; then
  IMG=${IMG::-1}
fi
echo $IMG

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}" | head -n1)

# Check if the container is running, and if not, start it first
container_status=$(docker inspect -f '{{.State.Status}}' "$containerid")
if [ "$container_status" == "exited" ]; then
  echo "Container is not running. Starting it now..."
  docker start "$containerid"
fi

docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES=`tput lines` -it ${containerid} bash
xhost -
