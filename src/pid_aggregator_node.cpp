#include <pid_control/pid_aggregator.h>
#include <au_core/sigint_handler.h>

const double RATE = 30;

int main(int argc, char** argv) {
  au_core::handleInterrupts(argc, argv, "pid_aggregator");

  pid_control::PidAggregator pidAggregator;

  ros::Rate r(RATE);
  while (!au_core::exitFlag) {
    ros::spinOnce();
    r.sleep();
  }

  pidAggregator.stop();
  ros::shutdown();
  return 0;
}
