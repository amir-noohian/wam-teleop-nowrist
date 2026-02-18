# Note
This version of code is for horizontal configuration of the WAM teleop setup.

# Wam Teleop
This package enables teleoperation between the 4DOF leader with the haptic wrist and the 7DOF follower. While this is a ros package, communication between WAMs does not use ros, and instead uses UDP. Ros is only used to publish the state of arm for easier data collection, and is not intended to receive any incoming messages or services to control the arm. 

## Build Instructions

Place this package in `<catkin_ws>/src/`.

By default, this package builds the leader and follower node. If you only wish to build the follower node you can configure this from the command line:
```bash
catkin_make --cmake-args -DBUILD_LEADER=OFF 
```
or set the option to `OFF` in `CMakeLists.txt`:
```bash
option(BUILD_LEADER "Build leader executable" OFF)
```
To build the leader node, the haptic_wrist library is required as a dependency, build and install instructions can be found [here](https://github.com/dmiller12/libhaptic_wrist).

## Run Instructions

`config/` contains the Barrett configuration files for the leader and follower. 
You can set the correct config file by using `source scripts/setup_leader.sh` and `source scripts/setup_follower.sh`. These will set the env variable `BARRETT_CONFIG_FILE` to the correct path. 
You may need to modify the bus port in `config/leader.conf` and `config/follower.conf` depending on the can interface. Note that these environment variables only persist for the current terminal session.

Each node has the same command line options:
```bash
rosrun wam_teleop leader [remoteHost] [recPort] [sendPort]
rosrun wam_teleop follower [remoteHost] [recPort] [sendPort]
```
Use `-h` or `--help` to see options description.

### Example

Source workspace in each terminal in `amir\catkin_ws`
```bash
source devel/setup.bash
```


In first terminl, source CAN in a terminal in `amir\catkin_ws\src\wam_teleop`
```bash
source scripts\can_init.sh
```

In a separate terminal session, start the master node with: `roscore`.

In a separate terminal session, start the leader in `amir\catkin_ws\src\wam_teleop`:
```bash
source scripts/setup_leader.sh
rosrun wam_teleop leader_nowrist 127.0.0.1 5555 5554
```
In another separate terminal session, start the follower in `amir\catkin_ws\src\wam_teleop`:
```bash
source scripts/setup_follower.sh
rosrun wam_teleop follower 127.0.0.1 5554 5555
```
Note the matching recPort and sendPort between leader and follower, that is, the leader receives on 5555 and the follower sends on 5555 and vice versa.

Once both nodes have started:

1) On the leader use `l` to go to the sync position.
2) On the follower use `l` to go to the sync position. Ensure both arms have reached the sync position before continuing.
3) Press enter to link leader
4) Press enter to link follower

The arm is now ready for user teleoperation.

To turn off, it is recommended to go through the following procedure to ensure proper thread and socket cleanup.
1) Return both wams to home position
2) On the leader, press `x` to exit the loop.
3) Shift idle the leader
4) Repeat for follower. Press `x` to exit the loop
5) Shift idle the follower.



### runnning DMP's

After linking preform the following:
1) Move the robots to the start position of demo
2) Press `s` on follower node to set that position
3) Each time you switch to `p` the code will expect that you start from the start position which you can get to by pressing `g` in the follower node.
4) Start ./scripts/dmp.py in the dmp repo and press `s` to start recording, move the wams then finish recording by typing enter
5) Move back to demo start with `g`
6) Press `r` to start replay in dmp code.
7) Press `p` to let teleop setup actually follow policy.

TODO: you should press `p` after `r` pretty quickly or the wam will jump. This should be fixed eventually.
TODO: policy is very stiff and you cant really instruct finetuning instructions 
