# LQR Controller

### To run:
* Run the simulator (`roslaunch underwater_vehicle_dynamics qualification.launch`)
* Run the state aggregator (`roslaunch au_sensors state_aggregator.launch`)
* Run the lqr controller (`roslaunch au_control lqr_controller.launch`)
* Publish the target state to `/dynamics/target` using `rqt`. We recommend sending the message at 10Hz.

* Note the orientation is in quaternions

* Example: To make the robot dive send the following message on `/dynamics/target` using `rqt` :
```
pose:
  position:
  # In meters
    x: 0.0
    y: 0.0
    z: 1.0
  orientation:
  # Quaterions
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
velocity:
  # In m/s
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
  # In rad/s
    x: 0.0
    y: 0.0
    z: 0.0
```

### Use the sim params
```
roslaunch au_control lqr_controller.launch params:=lqr_sim.yaml
```


### Save dynamic reconfigure:
```
rosrun dynamic_reconfigure dynparam dump /lqr_controller ~/au_everything/catkin_ws/src/au_control/params/lqr.yaml
```

### Diagnosing latency issues
```
# to see how many states we are dropping, check the difference between these two:
rostopic hz /dynamics/state
rostopic hz /motor/raw
```

## LQR Controller (Matlab Version)

### Setup:
* Install Matlab (instructions [here](https://drive.google.com/drive/folders/0AM-pbQTYL-n7Uk9PVA)) ensuring to install the Robotics System Toolbox add-on
* Install the Robotics System Toolbox Interface for ROS Custom Messages by running the `roboticsAddons` command in the matlab terminal and following the installation instructions
* Go to `~/au_everything/catkin_ws/src/canros` and rename `package.xml` to `not_package.xml`
* Run the ROS simulator (`roslaunch underwater_vehicle_dynamics qualification.launch`)
* Run the `rosgen.m` script and follow the setup instructions on the Matlab terminal. Ignore the error message for the `savepath` command. Make sure to restart Matlab.
* Once the code is running properly, go to `~/au_everything/catkin_ws/src/canros` and rename `not_package.xml` to `package.xml`
* NOTE: any time that a new ROS message type has been added the `rosgen.m` must be used to generate the ROS message on Matlab

### To run:
* Run the simulator (`roslaunch underwater_vehicle_dynamics qualification.launch`)
* Run the state aggregator (`roslaunch au_sensors state_aggregator.launch`)
* Run the `lqr_node.m` file on Matlab
* Publish the target state to `/dynamics/target` using `rqt`. We recommend sending the message at 10Hz.

* Note the orientation is in quaternions

## PID Controller

Subscribes to `/dynamics/state`, `/dynamics/target`.
Publishes to `/dynamics/error` and all PID controllers.

### To run:
* Make sure the state aggregator is running `roslaunch au_sensors state_aggregator.launch`
* Run the pid aggregator `roslaunch au_control pid_controllers.launch`
