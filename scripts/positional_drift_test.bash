
set_xy_pose() {
rostopic pub /dynamics/target au_core/DynamicsState -1 "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: $1
    y: $2
    z: 0.3
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
velocity:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"
  sleep 15
}

i="1"
echo "setting xy pose to 0"
set_xy_pose 0.0 0.0
echo "starting test for 4 iterations"
while [ $i -lt 5 ]
do

  set_xy_pose 0.8 0.0
  set_xy_pose 0.8 0.8
  set_xy_pose 0.0 0.8
  set_xy_pose 0.0 0.0

  echo "Iteration $i done, final estimated position:"
  rostopic echo /dynamics/state/pose/position -n 1
  i=$[$i+1]
  echo "-------------------------"
done
