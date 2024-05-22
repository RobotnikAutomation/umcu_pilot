#include <umcu_pilot/umcu_pilot.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "umcu_pilot");
  ros::NodeHandle n;

  UmcuPilot umcu_pilot(n);
  umcu_pilot.start();
}
