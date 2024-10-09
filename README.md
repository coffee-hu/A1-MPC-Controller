# Based on A1-QP-MPC-Controller with some modifications 
A1-QP-MPC-Controller See the warehouse https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller.git

# update 2024-10-9

## Added keyboard control codes： Mapped keyboard controls to gamepads

```
rosrun pub_joy_node pub_joy_node
```

## First time setup of the real robot:

### Build a base ros image. 
 
```
cd docker
bash build.bash a1_mpc_control_real melodic
```

### Create a dev container

```
bash run.bash a1_mpc_control_real:melodic
```
### Start the container

If you have already created a dev container, you can just run the following scripts to start the container, or use it to open another terminal of the container.

```
bash join.bash a1_mpc_control_real:melodic
```