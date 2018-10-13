## PID Controller
Must move the PID package into ~/au_everything/catkin_ws before running it.

Subscribes to `/dynamics/state`, `/dynamics/target`.
Publishes to `/dynamics/error` and all PID controllers.

### To run:
* Make sure the state aggregator is running `roslaunch au_sensors state_aggregator.launch`
* Run the pid aggregator `roslaunch au_control pid_controllers.launch`
