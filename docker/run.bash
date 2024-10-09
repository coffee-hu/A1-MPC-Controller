    docker run -d \
    --network host \
    --cap-add=IPC_LOCK --cap-add=sys_nice \
    -v /home/huzhanyong/A1-QP-MPC-Controller:/root/A1_ctrl_ws/src/A1_Ctrl \
    --device /dev/input \
    --name a1_cpp_ctrl_docker \
    a1_cpp_ctrl_image