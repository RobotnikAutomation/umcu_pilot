#ifndef _SERMAS_PILOT_
#define _SERMAS_PILOT_

#include <rcomponent/rcomponent.h>

// General includes
#include <actionlib/client/simple_action_client.h>
#include <math.h>

// Msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <odin_msgs/HMIBase.h>
#include <odin_msgs/ProxSensor.h>
#include <odin_msgs/RobotStatus.h>
#include <odin_msgs/RobotTask.h>
#include <odin_msgs/RTLSBase.h>
#include <odin_msgs/SmartboxStatus.h>
#include <robotnik_msgs/BatteryStatus.h>
#include <robotnik_msgs/ElevatorStatus.h>
#include <robotnik_msgs/StringStamped.h>
#include <std_msgs/String.h>

// Srvs
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

// Actions
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_simple_command_manager_msgs/RobotSimpleCommandAction.h>

class SermasPilot : public rcomponent::RComponent
{
public:
  SermasPilot(ros::NodeHandle h);
  ~SermasPilot() override;

protected:
  /*** RComponent Stuff ***/
  //! Setups all the ROS' Stuff
  int rosSetup() override;
  //! Shutdowns all the ROS' Stuff
  int rosShutdown() override;
  //! Reads data a publish several info into different topics
  void rosPublish() override;
  //! Reads params from params server
  void rosReadParams() override;
  //! Actions performed on init state
  void initState() override;
  //! Actions performed on standby state
  void standbyState() override;
  //! Actions performed on ready state
  void readyState() override;
  //! Actions performed on the emergency state
  void emergencyState() override;
  //! Actions performed on Failure state
  void failureState() override;
  /* RComponent Stuff !*/

  /*** ROS Stuff ***/
  //! Publishers
  ros::Publisher robot_status_pub_;
  string robot_status_pub_name_;
  ros::Publisher robot_result_pub_;
  string robot_result_pub_name_;
  ros::Publisher state_machine_state_pub_;
  std_msgs::String current_state_data_;

  //! Subscribers
  ros::Subscriber smartbox_sub_;
  string smartbox_sub_name_;
  ros::Subscriber rtls_sub_;
  string rtls_sub_name_;
  ros::Subscriber hmi_sub_;
  string hmi_sub_name_;
  ros::Subscriber elevator_sub_;
  string elevator_sub_name_;
  ros::Subscriber battery_sub_;
  string battery_sub_name_;
  ros::Subscriber pose_sub_;
  string pose_sub_name_;

  //! Services Servers
  // _Pick Up_ Mission
  ros::ServiceServer pickup_mission_received_srv_;      // 1 --> 2
  ros::ServiceServer elevator_down_srv_;                // 2 --> 3
  ros::ServiceServer rack_position_received_srv_;       // 3 --> 4
  ros::ServiceServer correct_position_srv_;             // 4 --> 3 (false), 5 (true)
  ros::ServiceServer arrived_at_rack_srv_;              // 5 --> 6
  ros::ServiceServer rack_picked_srv_;                  // 6 --> 7
  ros::ServiceServer go_from_first_to_second_room_srv_; // 7 --> 8
  ros::ServiceServer arrived_at_second_room_srv_;       // 8 --> 9
  ros::ServiceServer release_rack_srv_;                 // 9, 11 --> 12 (false), 14 (true)
  ros::ServiceServer go_from_second_to_next_room_srv_;  // 9 --> 10
  ros::ServiceServer arrived_at_next_room_srv_;         // 10 --> 11
  ros::ServiceServer go_to_next_room_srv_;              // 11 --> 10
  ros::ServiceServer rack_released_srv_;                // 14 --> 15
  ros::ServiceServer rack_homed_srv_;                   // 12 --> 13
  ros::ServiceServer rack_placed_srv_;                  // 13 --> 15
  ros::ServiceServer arrived_at_home_srv_;              // 15 (and 23) --> 1

  // _Recharge_ Mission
  ros::ServiceServer recharge_mission_received_srv_; // 1 --> 16
  ros::ServiceServer goal_calculated_srv_;           // 18 --> 19
  ros::ServiceServer rack_charged_srv_;              // 21 --> 22
  // ros::ServiceServer elevator_down_srv_;          // 16 --> 17
  // ros::ServiceServer rack_position_received_srv_; // 17 --> 18
  // ros::ServiceServer arrived_at_rack_srv_;        // 19 --> 20
  // ros::ServiceServer rack_picked_srv_;            // 20 --> 21
  // ros::ServiceServer rack_released_srv_;          // 22 --> 23
  // ros::ServiceServer arrived_at_home_srv_;        // 23 (and 15) --> 1

  //! Services Clients
  // ros::ServiceClient out_of_battery_client_;
  // ros::ServiceClient rack_position_received_client_;
  // ros::ServiceClient goal_calculated_client_;
  // ros::ServiceClient arrived_at_rack_client_;
  // ros::ServiceClient rack_picked_client_;
  // ros::ServiceClient arrived_at_home_client_;

  //! Action Clients
  std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_ac_;
  move_base_msgs::MoveBaseGoal move_base_goal_;

  std::shared_ptr<actionlib::SimpleActionClient<robot_simple_command_manager_msgs::RobotSimpleCommandAction>> command_sequencer_ac_;
  robot_simple_command_manager_msgs::RobotSimpleCommandGoal command_sequencer_goal_;

  //! Callbacks
  //! Subscription Callbacks
  void smartboxSubCb(const odin_msgs::SmartboxStatus::ConstPtr &msg);
  void rtlsSubCb(const odin_msgs::RTLSBase::ConstPtr &msg);
  void hmiSubCb(const odin_msgs::HMIBase::ConstPtr &msg);
  void elevatorSubCb(const robotnik_msgs::ElevatorStatus::ConstPtr &msg);
  void batterySubCb(const robotnik_msgs::BatteryStatus::ConstPtr &msg);
  void poseSubCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  //! Service Callbacks
  // _Pick Up Mission
  // 1. WAITING_FOR_MISSION --> 2. CHECKING_ELEVATOR
  bool pickupMissionReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 2. CHECKING_ELEVATOR --> 3. GETTING_RACK_POSITION, or 16. CHECKING_ELEVATOR --> 17. GETTING_RACK_POSITION
  bool elevatorDownServiceCb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
  // 3. GETTING_RACK_POSITION --> 4. CHECKING_RACK_POSITION, or 17. GETTING_RACK_POSITION --> 18. CALCULATING_GOAL
  bool rackPositionReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 4. CHECKING_RACK_POSITION --> 3. GETTING_RACK_POSITION or 5. NAVIGATING_TO_RACK
  bool correctPositionServiceCb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
  // 5. NAVIGATING_TO_RACK --> 6. PICKING_RACK
  bool arrivedAtRackServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 6. PICKING_RACK --> 7. WAITING_IN_FIRST_ROOM
  bool rackPickedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 7. WAITING_IN_FIRST_ROOM --> 8. NAVIGATING_TO_SECOND_ROOM
  bool goFromFirstToSecondRoomServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 8. NAVIGATING_TO_SECOND_ROOM --> 9. WAITING_IN_SECOND_ROOM
  bool arrivedAtSecondRoomServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 9. WAITING_IN_SECOND ROOM or 11. WAITING_IN_NEXT_ROOM --> 12. HOMING_RACK or 14. RELEASING_RACK
  bool releaseRackServiceCb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
  // 9. WAITING_IN_SECOND_ROOM --> 10. NAVIGATING_TO_NEXT_ROOM
  bool goFromSecondToNextRoomServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 10. NAVIGATING_TO_NEXT_ROOM --> 11. WAITING_IN_NEXT_ROOM
  bool arrivedAtNextRoomServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 11. WAITING_IN_NEXT_ROOM --> 10. NAVIGATING_TO_NEXT_ROOM
  bool goToNextRoomServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 14. RELEASING_RACK --> 15. NAVIGATING_TO_HOME
  bool rackReleasedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 12. HOMING_RACK --> 13. PLACING_RACK
  bool rackHomedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 13. PLACING_RACK --> 15. NAVIGATING_TO_HOME
  bool rackPlacedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 15. NAVIGATING_TO_HOME (also 23) --> 1. WAITING_FOR_MISSION
  bool arrivedAtHomeServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  // _Recharge_ Mission
  // 1. WAITING_FOR_MISSION --> 16. CHECKING_ELEVATOR
  bool rechargeMissionReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 18. CALCULATING_GOAL --> 19. NAVIGATING_TO_RACK
  bool goalCalculatedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
  // 21. CHARGING_RACK --> 22. RELEASING_RACK
  bool rackChargedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  //! Action Callbacks
  void moveBaseResultCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
  void commandSequencerResultCb(const actionlib::SimpleClientGoalState &state, const robot_simple_command_manager_msgs::RobotSimpleCommandResultConstPtr &result);
  /* ROS Stuff !*/

  /*** SermasPilot Stuff ***/
  std_msgs::String status_;
  string current_state_;
  string previous_state_;
  float battery_status_{0.0};
  geometry_msgs::PoseWithCovarianceStamped pose_;

  //! State Machine
  void runRobotStateMachine();
  void changeState(const string &next_state, const string &additional_information);

  //! 1. WAITING_FOR_MISSION
  void waitingForMissionState();
  bool mission_received_;
  float poi_x_{0.0};
  float poi_y_{0.0};
  float poi_rot_z_{0.0};
  float poi_rot_w_{1.0};
  float home_x_{0.0};
  float home_y_{0.0};

  //! 2. CHECKING_ELEVATOR, or 16. CHECKING_ELEVATOR
  void checkingElevatorState();

  //! 3. GETTING_RACK_POSITION, or 17. GETTING_RACK_POSITION
  void gettingRackPositionState();
  double rack_x_{0.0};
  double rack_y_{0.0};
  double rack_z_{0.0};
  double x1_{1.59983614424};
  double y1_{1.7909510717};
  double z1_{0.0};
  double x2_{12.614839128};
  double y2_{2.96726033821};
  double z2_{0.0};
  double x_goal_{0.0};
  double y_goal_{0.0};
  double z_goal_{0.0};
  double z_orient_goal{0.0};
  double w_orient_goal{0.0};

  // 4. CHECKING_RACK_POSITION
  void checkingRackPositionState();
  double room1_x_{0.0};
  double room1_y_{0.0};

  //! 5. NAVIGATING_TO_RACK, or 19. NAVIGATING_TO_RACK
  void navigatingToRackState();
  bool navigation_command_sent_;

  //! 6. PICKING_RACK, or 20. PICKING_RACK
  void pickingRackState();
  string pick_sequence_;
  bool sequence_sent_;

  //! 7. WAITING_IN_FIRST_ROOM
  void waitingInFirstRoomState();
  // TODO: change variable names
  float lab_pos_x_{0.0};
  float lab_pos_y_{0.0};
  float lab_pos_z_{0.0};
  float lab_ori_x_{0.0};
  float lab_ori_y_{0.0};
  float lab_ori_z_{0.0};
  float lab_ori_w_{0.0};

  //! 8. NAVIGATING_TO_SECOND_ROOM
  void navigatingToSecondRoomState();

  //! 9. WAITING_IN_SECOND_ROOM
  void waitingInSecondRoomState();

  //! 10. NAVIGATING_TO_NEXT_ROOM
  void navigatingToNextRoomState();

  //! 11. WAITING_IN_NEXT_ROOM
  void waitingInNextRoomState();

  //! 12. HOMING_RACK
  void homingRackState();

  //! 13. PLACING_RACK
  void placingRackState();
  string place_sequence_;

  //! 14. RELEASING_RACK, or 22. RELEASING_RACK
  void releasingRackState();
  string release_sequence_;

  //! 15. NAVIGATING_TO_HOME, or 23. NAVIGATING_TO_HOME
  void navigatingToHomeState();

  //! 18. CALCULATING_GOAL
  void calculatingGoalState();

  //! 21. CHARGING_RACK
  void chargingRackState();
  /* SermasPilot Stuff !*/
};

#endif // _SERMAS_PILOT_
