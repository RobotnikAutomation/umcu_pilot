#include <sermas_pilot/sermas_pilot.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sermas_pilot");
  ros::NodeHandle n;
  
  MulPilot sermas_pilot(n);
  sermas_pilot.start();
}
