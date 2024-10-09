# Deal with args
if [ $# -eq 0 ]
then
  # default image and tag
  image_name="a1_mpc_control_real"
  tag="melodic"
elif [ $# -eq 1 ]
then
  # image from arg, default tag
  image_name=$1
  tag="melodic"
else 
  # image and tag from args
  image_name=$1
  tag=$2
fi

user_id=$(id -u)

# Use same name as source directory so can tab-complete on run.bash
image_plus_tag="$image_name:$tag"

echo "Building $image_name"

docker build --rm -t \
  $image_plus_tag \
  --build-arg user_id=$user_id \
  --build-arg host_name=$USER \
  --build-arg SSH_PRIVATE_KEY="$ssh_private_key_content" \
  --build-arg BUILD_DATE=$(date -u +'%Y-%m-%dT%H:%M:%SZ') \
  -f Dockerfile .

echo "Built $image_plus_tag"
