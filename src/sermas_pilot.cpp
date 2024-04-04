#include <sermas_pilot/sermas_pilot.h>

SermasPilot::SermasPilot(ros::NodeHandle h) : rcomponent::RComponent(h)
{
  component_name.assign(pnh_.getNamespace());
  rosReadParams();
}

SermasPilot::~SermasPilot()
{
}

void SermasPilot::rosReadParams()
{
  bool required = true;
  bool not_required = false;

  readParam(pnh_, "desired_freq", desired_freq_, 10.0, not_required);
  readParam(pnh_, "robot_status_pub", robot_status_pub_name_, "/sermas_pilot/robot_status", required);
  readParam(pnh_, "robot_result_pub", robot_result_pub_name_, "/sermas_pilot/robot_result", required);
  readParam(pnh_, "rtls_sub", rtls_sub_name_, "/sermas_pilot/rtls", required);
  readParam(pnh_, "smartbox_sub", smartbox_sub_name_, "/sermas_pilot/smartbox", required);
  readParam(pnh_, "hmi_sub", hmi_sub_name_, "/sermas_pilot/hmi", required);
  readParam(pnh_, "elevator_sub", elevator_sub_name_, "/robot/robotnik_base_control/elevator_status", required);
  readParam(pnh_, "battery_sub", battery_sub_name_, "/robot/battery_estimator/data", required);
  readParam(pnh_, "pose_sub", pose_sub_name_, "/robot/amcl_pose", required);
  readParam(pnh_, "pick_sequence", pick_sequence_, "TEST_ALLINEAMENTO_RUOTE", required);
  readParam(pnh_, "place_sequence", place_sequence_, "PLACE_SEQUENCE", required);
  readParam(pnh_, "release_sequence", release_sequence_, "RELEASE_AND_HOME", required);
  if (readParam(pnh_, "locations", locations_, locations_, required))
  {
    loadLocationParameters(locations_);
  }
  else
  {
    RCOMPONENT_ERROR_STREAM("Couldn't read locations!");
  }
  if (readParam(pnh_, "rtls_ids", rtls_ids_, rtls_ids_, required))
  {
    loadRtlsIds(rtls_ids_);
  }
  else
  {
    RCOMPONENT_ERROR_STREAM("Couldn't read locations!");
  }
}

int SermasPilot::rosSetup()
{
  RComponent::rosSetup();

  bool required = true;
  bool not_required = false;

  /*** ROS Stuff ***/
  //! Publishers
  robot_status_pub_ = pnh_.advertise<odin_msgs::RobotStatus>(robot_status_pub_name_, 10);
  robot_result_pub_ = pnh_.advertise<odin_msgs::RobotTask>(robot_result_pub_name_, 10);
  state_machine_state_pub_ = pnh_.advertise<std_msgs::String>("/sermas_pilot/state_machine", 10);

  //! Subscribers
  smartbox_sub_ = nh_.subscribe<odin_msgs::SmartboxStatus>(smartbox_sub_name_, 10, &SermasPilot::smartboxSubCb, this);
  addTopicsHealth(&smartbox_sub_, smartbox_sub_name_, 50.0, not_required);
  rtls_sub_ = nh_.subscribe<odin_msgs::RTLSBase>(rtls_sub_name_, 10, &SermasPilot::rtlsSubCb, this);
  addTopicsHealth(&rtls_sub_, rtls_sub_name_, 50.0, not_required);
  hmi_sub_ = nh_.subscribe<odin_msgs::HMIBase>(hmi_sub_name_, 10, &SermasPilot::hmiSubCb, this);
  addTopicsHealth(&hmi_sub_, hmi_sub_name_, 50.0, not_required);
  elevator_sub_ = nh_.subscribe<robotnik_msgs::ElevatorStatus>(elevator_sub_name_, 10, &SermasPilot::elevatorSubCb, this);
  addTopicsHealth(&elevator_sub_, elevator_sub_name_, 50.0, not_required);
  battery_sub_ = nh_.subscribe<robotnik_msgs::BatteryStatus>(battery_sub_name_, 10, &SermasPilot::batterySubCb, this);
  addTopicsHealth(&battery_sub_, battery_sub_name_, 50.0, not_required);
  pose_sub_ = nh_.subscribe<>(pose_sub_name_, 10, &SermasPilot::poseSubCb, this);
  addTopicsHealth(&pose_sub_, pose_sub_name_, 50.0, not_required);

  //! Service Servers
  // _Pick Up_ Mission
  pickup_mission_received_srv_ = pnh_.advertiseService("/sermas_pilot/pickup_mission_received", &SermasPilot::pickupMissionReceivedServiceCb, this);
  elevator_down_srv_ = pnh_.advertiseService("/sermas_pilot/elevator_down", &SermasPilot::elevatorDownServiceCb, this);
  rack_position_received_srv_ = pnh_.advertiseService("/sermas_pilot/rack_position_received", &SermasPilot::rackPositionReceivedServiceCb, this);
  correct_position_srv_ = pnh_.advertiseService("/sermas_pilot/correct_position", &SermasPilot::correctPositionServiceCb, this);
  arrived_at_rack_srv_ = pnh_.advertiseService("/sermas_pilot/arrived_at_rack", &SermasPilot::arrivedAtRackServiceCb, this);
  rack_picked_srv_ = pnh_.advertiseService("/sermas_pilot/rack_picked", &SermasPilot::rackPickedServiceCb, this);
  go_from_first_to_second_room_srv_ = pnh_.advertiseService("/sermas_pilot/go_from_first_to_second_room", &SermasPilot::goFromFirstToSecondRoomServiceCb, this);
  arrived_at_second_room_srv_ = pnh_.advertiseService("/sermas_pilot/arrived_at_second_room", &SermasPilot::arrivedAtSecondRoomServiceCb, this);
  release_rack_srv_ = pnh_.advertiseService("/sermas_pilot/release_rack", &SermasPilot::releaseRackServiceCb, this);
  go_from_second_to_next_room_srv_ = pnh_.advertiseService("/sermas_pilot/go_from_second_to_next_room", &SermasPilot::goFromSecondToNextRoomServiceCb, this);
  arrived_at_next_room_srv_ = pnh_.advertiseService("/sermas_pilot/arrived_at_next_room", &SermasPilot::arrivedAtNextRoomServiceCb, this);
  go_to_next_room_srv_ = pnh_.advertiseService("/sermas_pilot/go_to_next_room", &SermasPilot::goToNextRoomServiceCb, this);
  rack_released_srv_ = pnh_.advertiseService("/sermas_pilot/rack_released", &SermasPilot::rackReleasedServiceCb, this);
  rack_homed_srv_ = pnh_.advertiseService("/sermas_pilot/rack_homed", &SermasPilot::rackHomedServiceCb, this);
  rack_placed_srv_ = pnh_.advertiseService("/sermas_pilot/rack_placed", &SermasPilot::rackPlacedServiceCb, this);
  arrived_at_home_srv_ = pnh_.advertiseService("/sermas_pilot/arrived_at_home", &SermasPilot::arrivedAtHomeServiceCb, this);

  // _Recharge_ Mission
  recharge_mission_received_srv_ = pnh_.advertiseService("/sermas_pilot/recharge_mission_received", &SermasPilot::rechargeMissionReceivedServiceCb, this);
  goal_calculated_srv_ = pnh_.advertiseService("/sermas_pilot/goal_calculated", &SermasPilot::goalCalculatedServiceCb, this);
  rack_charged_srv_ = pnh_.advertiseService("/sermas_pilot/rack_charged", &SermasPilot::rackChargedServiceCb, this);

  //! Service Clients
  // rack_position_received_client_ = pnh_.serviceClient<std_srvs::Trigger>("/sermas_pilot/rack_position_received");
  // goal_calculated_client_ = pnh_.serviceClient<std_srvs::Trigger>("/sermas_pilot/goal_calculated");
  // arrived_at_rack_client_ = pnh_.serviceClient<std_srvs::Trigger>("/sermas_pilot/arrived_at_rack");
  // rack_picked_client_ = pnh_.serviceClient<std_srvs::Trigger>("/sermas_pilot/rack_picked");
  // arrived_at_home_client_ = pnh_.serviceClient<std_srvs::Trigger>("/sermas_pilot/arrived_at_home");

  //! Action Clients
  move_base_ac_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(pnh_, "/robot/move_base", true);
  command_sequencer_ac_ = std::make_shared<actionlib::SimpleActionClient<robot_simple_command_manager_msgs::RobotSimpleCommandAction>>(pnh_, "/robot/command_sequencer/action", true);
  /* ROS Stuff !*/

  return rcomponent::OK;
}

int SermasPilot::rosShutdown()
{
  return RComponent::rosShutdown();
}

void SermasPilot::rosPublish()
{
  RComponent::rosPublish();

  if (getState() == robotnik_msgs::State::READY_STATE)
  {
    // Publishers
    // robot_status_.data.battery = battery_status_;
    robot_status_.data.status = current_state_;
    // robot_status.data.pose = pose_;
    robot_status_pub_.publish(robot_status_);

    current_state_data_.data = current_state_;
    state_machine_state_pub_.publish(current_state_data_);
  }
}

void SermasPilot::initState()
{
  RComponent::initState();

  current_state_ = "1_WAITING_FOR_MISSION";
  previous_state_ = "";

  navigation_command_sent_ = false;
  sequence_sent_ = false;

  // robot_status_.data.battery = battery_status_;
  // robot_status_.data.status = current_state_;
  // robot_status_.data.pose = pose_;
  // robot_status_pub_.publish(robot_status_);

  // current_state_data_.data = current_state_;
  // state_machine_state_pub_.publish(current_state_data_);

  switchToState(robotnik_msgs::State::STANDBY_STATE);
}

void SermasPilot::standbyState()
{
  if (checkTopicsHealth() == false)
  {
    switchToState(robotnik_msgs::State::EMERGENCY_STATE);
  }
  else
  {
    switchToState(robotnik_msgs::State::READY_STATE);
  }
}

void SermasPilot::readyState()
{
  if (checkTopicsHealth() == false)
  {
    switchToState(robotnik_msgs::State::EMERGENCY_STATE);
  }

  runRobotStateMachine();
}

void SermasPilot::emergencyState()
{
  if (checkTopicsHealth() == true)
  {
    switchToState(robotnik_msgs::State::STANDBY_STATE);
  }
}

void SermasPilot::failureState()
{
  RComponent::failureState();
}

/*** State Machine ***/
void SermasPilot::runRobotStateMachine()
{
  if (current_state_ == "1_WAITING_FOR_MISSION")
  {
    waitingForMissionState();
  }
  else if (current_state_ == "2_CHECKING_ELEVATOR" || current_state_ == "16_CHECKING_ELEVATOR")
  {
    checkingElevatorState();
  }
  else if (current_state_ == "3_GETTING_RACK_POSITION" || current_state_ == "17_GETTING_RACK_POSITION")
  {
    gettingRackPositionState();
  }
  else if (current_state_ == "4_CHECKING_RACK_POSITION")
  {
    checkingRackPositionState();
  }
  else if (current_state_ == "5_NAVIGATING_TO_RACK" || current_state_ == "19_NAVIGATING_TO_RACK")
  {
    navigatingToRackState();
  }
  else if (current_state_ == "6_PICKING_RACK" || current_state_ == "20_PICKING_RACK")
  {
    pickingRackState();
  }
  else if (current_state_ == "7_WAITING_IN_FIRST_ROOM")
  {
    waitingInFirstRoomState();
  }
  else if (current_state_ == "8_NAVIGATING_TO_SECOND_ROOM")
  {
    navigatingToSecondRoomState();
  }
  else if (current_state_ == "9_WAITING_IN_SECOND_ROOM")
  {
    waitingInSecondRoomState();
  }
  else if (current_state_ == "10_NAVIGATING_TO_NEXT_ROOM")
  {
    navigatingToNextRoomState();
  }
  else if (current_state_ == "11_WAITING_IN_NEXT_ROOM")
  {
    waitingInNextRoomState();
  }
  else if (current_state_ == "12_HOMING_RACK")
  {
    homingRackState();
  }
  else if (current_state_ == "13_PLACING_RACK")
  {
    placingRackState();
  }
  else if (current_state_ == "14_RELEASING_RACK" || current_state_ == "22_RELEASING_RACK")
  {
    releasingRackState();
  }
  else if (current_state_ == "15_NAVIGATING_TO_HOME" || current_state_ == "23_NAVIGATING_TO_HOME")
  {
    navigatingToHomeState();
  }
  else if (current_state_ == "18_CALCULATING_GOAL")
  {
    calculatingGoalState();
  }
  else if (current_state_ == "21_CHARGING_RACK")
  {
    chargingRackState();
  }
  else
  {
    ROS_WARN("The current state is unknown!");
  }
}

void SermasPilot::changeState(const string &next_state, const string &additional_information)
{
  RCOMPONENT_WARN_STREAM(additional_information);
  RCOMPONENT_WARN_STREAM(current_state_ << " --> " << next_state);

  string temp_state = current_state_;
  current_state_ = next_state;
  previous_state_ = temp_state;

  navigation_command_sent_ = false;
  sequence_sent_ = false;
}
/* State Machine !*/

/*** States ***/

//! 1_WAITING_FOR_MISSION
// The RB-1 waits for a mission in the HOME ROOM
void SermasPilot::waitingForMissionState()
{
  RCOMPONENT_INFO_STREAM("1_WAITING_FOR_MISSION");
}

//! 2_CHECKING_ELEVATOR, or 16_CHECKING_ELEVATOR
// The RB-1 checks the elevator position (up or down)
void SermasPilot::checkingElevatorState()
{
  if (current_state_ == "2_CHECKING_ELEVATOR")
  {
    RCOMPONENT_INFO_STREAM("2_CHECKING_ELEVATOR");
  }
  else if (current_state_ == "16_CHECKING_ELEVATOR")
  {
    RCOMPONENT_INFO_STREAM("16_CHECKING_ELEVATOR");
  }
}

//! 3_GETTING_RACK_POSITION, or 17_GETTING_RACK_POSITION
// The RB-1 gets the approximate rack position from the RTLS
void SermasPilot::gettingRackPositionState()
{
  if (current_state_ == "3_GETTING_RACK_POSITION")
  {
    RCOMPONENT_INFO_STREAM("3_GETTING_RACK_POSITION");
  }
  else if (current_state_ == "17_GETTING_RACK_POSITION")
  {
    RCOMPONENT_INFO_STREAM("17_GETTING_RACK_POSITION");
  }
}

//! 4_CHECKING_RACK_POSITION
// The RB-1 checks if the rack is in the correct docking station (in ROOM 1)
void SermasPilot::checkingRackPositionState()
{
  RCOMPONENT_INFO_STREAM("4_CHECKING_RACK_POSITION");

  double distance = std::sqrt(std::pow(rack_x_ - room_1_docking_x_, 2) + std::pow(rack_y_ - room_1_docking_y_, 2));
  if (distance < 1.0)
  {
    RCOMPONENT_INFO_STREAM("The rack is closer than 1 meter to the _a priori_ known position of the docking station in ROOM 1.");
  }
  else
  {
    RCOMPONENT_INFO_STREAM("The rack is not closer than 1 meter to the _a priori known position of the docking station in ROOM 1.");
  }

  std_srvs::SetBool::Request correct_position_srv_request;
  std_srvs::SetBool::Response correct_position_srv_response;
  correct_position_srv_request.data = (distance < 1.0);

  if (correctPositionServiceCb(correct_position_srv_request, correct_position_srv_response))
  {
    if (correct_position_srv_response.success)
    {
      RCOMPONENT_INFO_STREAM("Successfully switched from 4_CHECKING_RACK_POSITION to 5_NAVIGATING_TO_RACK");
    }
    else
    {
      RCOMPONENT_WARN_STREAM("Failed to switch from 4_CHECKING_RACK_POSITION to 5_NAVIGATING_TO_RACK: " << correct_position_srv_response.message.c_str());
    }
  }
  else
  {
    RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/correct_position");
  }
}

//! 5_NAVIGATING_TO_RACK, or 19_NAVIGATING_TO_RACK
// The RB-1 navigates to the _a priori_ known pre-pick position of the rack
void SermasPilot::navigatingToRackState()
{
  // If we are in the 5_NAVIGATING_TO_RACK state, we are navigating to the rack in the HOME ROOM
  if (current_state_ == "5_NAVIGATING_TO_RACK")
  {
    RCOMPONENT_INFO_STREAM("5_NAVIGATING_TO_RACK");
    if (!navigation_command_sent_)
    {
      RCOMPONENT_INFO_STREAM("Sending command to navigate to the rack...");
      RCOMPONENT_INFO_STREAM("x: " << room_1_pre_pick_x_ << ", y: " << room_1_pre_pick_y_);

      move_base_goal_.target_pose.header.stamp = ros::Time::now();
      move_base_goal_.target_pose.header.frame_id = "robot_map";
      move_base_goal_.target_pose.pose.position.x = room_1_pre_pick_x_;
      move_base_goal_.target_pose.pose.position.y = room_1_pre_pick_y_;
      move_base_goal_.target_pose.pose.position.z = room_1_pre_pick_z_;
      move_base_goal_.target_pose.pose.orientation.x = room_1_pre_pick_rot_x_;
      move_base_goal_.target_pose.pose.orientation.y = room_1_pre_pick_rot_y_;
      move_base_goal_.target_pose.pose.orientation.z = room_1_pre_pick_rot_z_;
      move_base_goal_.target_pose.pose.orientation.w = room_1_pre_pick_rot_w_;
      move_base_ac_->sendGoal(move_base_goal_, boost::bind(&SermasPilot::moveBaseResultCb, this, _1, _2));

      navigation_command_sent_ = true;
    }
  }
  // If we are in the 19_NAVIGATING_TO_RACK state, we are navigating to the rack in the calcultad goal
  else if (current_state_ == "19_NAVIGATING_TO_RACK")
  {
    RCOMPONENT_INFO_STREAM("19_NAVIGATING_TO_RACK");
    if (!navigation_command_sent_)
    {
      RCOMPONENT_INFO_STREAM("Sending command to navigate to the rack...");
      RCOMPONENT_INFO_STREAM("x: " << room_1_pre_pick_x_ << ", y: " << room_1_pre_pick_y_);

      move_base_goal_.target_pose.header.stamp = ros::Time::now();
      move_base_goal_.target_pose.header.frame_id = "robot_map";
      move_base_goal_.target_pose.pose.position.x = x_goal_;
      move_base_goal_.target_pose.pose.position.y = y_goal_;
      move_base_goal_.target_pose.pose.position.z = z_goal_;
      move_base_goal_.target_pose.pose.orientation.x = x_orient_goal_;
      move_base_goal_.target_pose.pose.orientation.y = y_orient_goal_;
      move_base_goal_.target_pose.pose.orientation.z = z_orient_goal_;
      move_base_goal_.target_pose.pose.orientation.w = w_orient_goal_;
      move_base_ac_->sendGoal(move_base_goal_, boost::bind(&SermasPilot::moveBaseResultCb, this, _1, _2));

      navigation_command_sent_ = true;
    }
  }
}

//! 6_PICKING_RACK, or 20_PICKING_RACK
// The RB-1 picks up the rack -- which should be in the docking station
void SermasPilot::pickingRackState()
{
  if (current_state_ == "6_PICKING_RACK")
  {
    RCOMPONENT_INFO_STREAM("6_PICKING_RACK");
  }
  else if (current_state_ == "20_PICKING_RACK")
  {
    RCOMPONENT_INFO_STREAM("20_PICKING_RACK");
  }

  if (!sequence_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending sequence to pick the rack...");

    // TODO during pilot: Set correct command
    command_sequencer_goal_.command.command = pick_sequence_;
    command_sequencer_ac_->sendGoal(command_sequencer_goal_, boost::bind(&SermasPilot::commandSequencerResultCb, this, _1, _2));

    sequence_sent_ = true;
  }
}

//! 7_WAITING_IN_FIRST_ROOM
// The RB-1 waits in the first room until it is told to navigate to the next room by the hospital staff
void SermasPilot::waitingInFirstRoomState()
{
  RCOMPONENT_INFO_STREAM("7_WAITING_IN_FIRST_ROOM");
}

//! 8_NAVIGATING_TO_SECOND_ROOM
// The RB-1 navigates to the second room
void SermasPilot::navigatingToSecondRoomState()
{
  RCOMPONENT_INFO_STREAM("8_NAVIGATING_TO_SECOND_ROOM");
  if (!navigation_command_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending command to navigate to the second room...");
    RCOMPONENT_INFO_STREAM("x: " << room_2_place_x_ << ", y: " << room_2_place_y_);

    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.header.frame_id = "robot_map";
    move_base_goal_.target_pose.pose.position.x = room_2_place_x_;
    move_base_goal_.target_pose.pose.position.y = room_2_place_y_;
    move_base_goal_.target_pose.pose.position.z = room_2_place_z_;
    move_base_goal_.target_pose.pose.orientation.x = room_2_place_rot_x_;
    move_base_goal_.target_pose.pose.orientation.y = room_2_place_rot_y_;
    move_base_goal_.target_pose.pose.orientation.z = room_2_place_rot_z_;
    move_base_goal_.target_pose.pose.orientation.w = room_2_place_rot_w_;
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&SermasPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}

//! 9_WAITING_IN_SECOND_ROOM
// The RB-1 waits in the lab
void SermasPilot::waitingInSecondRoomState()
{
  RCOMPONENT_INFO_STREAM("9_WAITING_IN_SECOND_ROOM");
}

//! 10_NAVIGATING_TO_NEXT_ROOM
// The RB-1 navigates to the next room
void SermasPilot::navigatingToNextRoomState()
{
  RCOMPONENT_INFO_STREAM("10_NAVIGATING_TO_NEXT_ROOM");
  if (!navigation_command_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending command to navigate to the next room...");
    RCOMPONENT_INFO_STREAM("x: " << next_room_x_ << ", y: " << next_room_y_);

    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.header.frame_id = "robot_map";
    move_base_goal_.target_pose.pose.position.x = next_room_x_;
    move_base_goal_.target_pose.pose.position.y = next_room_y_;
    move_base_goal_.target_pose.pose.position.z = next_room_z_;
    move_base_goal_.target_pose.pose.orientation.x = next_room_rot_x_;
    move_base_goal_.target_pose.pose.orientation.y = next_room_rot_y_;
    move_base_goal_.target_pose.pose.orientation.z = next_room_rot_z_;
    move_base_goal_.target_pose.pose.orientation.w = next_room_rot_w_;
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&SermasPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}

//! 11_WAITING_IN_NEXT_ROOM
// The RB-1 waits in the next room
void SermasPilot::waitingInNextRoomState()
{
  RCOMPONENT_INFO_STREAM("11_WAITING_IN_NEXT_ROOM");
}

//! 12_HOMING_RACK
// The RB-1 brings back the rack to the HOME ROOM
void SermasPilot::homingRackState()
{
  RCOMPONENT_INFO_STREAM("12_HOMING_RACK");
  if (!navigation_command_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending command to navigate to the HOME ROOM...");

    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.header.frame_id = "robot_map";
    move_base_goal_.target_pose.pose.position.x = home_place_x_;
    move_base_goal_.target_pose.pose.position.y = home_place_y_;
    move_base_goal_.target_pose.pose.position.z = home_place_z_;
    move_base_goal_.target_pose.pose.orientation.x = home_place_rot_x_;
    move_base_goal_.target_pose.pose.orientation.y = home_place_rot_y_;
    move_base_goal_.target_pose.pose.orientation.z = home_place_rot_z_;
    move_base_goal_.target_pose.pose.orientation.w = home_place_rot_w_;
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&SermasPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}

//! 13_PLACING_RACK
// The RB-1 places the rack
void SermasPilot::placingRackState()
{
  RCOMPONENT_INFO_STREAM("13_PLACING_RACK");
  if (!sequence_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending sequence to place the rack...");

    // TODO during pilot: Set correct command
    command_sequencer_goal_.command.command = place_sequence_;
    command_sequencer_ac_->sendGoal(command_sequencer_goal_, boost::bind(&SermasPilot::commandSequencerResultCb, this, _1, _2));

    sequence_sent_ = true;
  }
}

//! 14_RELEASING_RACK, or 22_RELEASING_RACK
// The RB-1 releases the rack
void SermasPilot::releasingRackState()
{
  if (current_state_ == "14_RELEASING_RACK")
  {
    RCOMPONENT_INFO_STREAM("14_RELEASING_RACK");
  }
  else if (current_state_ == "22_RELEASING_RACK")
  {
    RCOMPONENT_INFO_STREAM("22_RELEASING_RACK");
  }

  if (!sequence_sent_)
  {
    RCOMPONENT_INFO_STREAM("Sending sequence to release the rack...");

    // TODO during pilot: Set correct command
    command_sequencer_goal_.command.command = release_sequence_;
    command_sequencer_ac_->sendGoal(command_sequencer_goal_, boost::bind(&SermasPilot::commandSequencerResultCb, this, _1, _2));

    sequence_sent_ = true;
  }
}

//! 15_NAVIGATING_TO_HOME, or 23_NAVIGATING_TO_HOME
// The RB-1 navigates to the HOME ROOM
void SermasPilot::navigatingToHomeState()
{
  if (current_state_ == "15_NAVIGATING_TO_HOME")
  {
    RCOMPONENT_INFO_STREAM("15_NAVIGATING_TO_HOME");
  }
  else if (current_state_ == "23_NAVIGATING_TO_HOME")
  {
    RCOMPONENT_INFO_STREAM("23_NAVIGATING_TO_HOME");
  }

  if (!navigation_command_sent_)
  {
    // TODO during pilot: Set correct coordinates
    RCOMPONENT_INFO_STREAM("Sending command to navigate to the HOME ROOM...");

    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.header.frame_id = "robot_map";
    move_base_goal_.target_pose.pose.position.x = home_robot_x_;
    move_base_goal_.target_pose.pose.position.y = home_robot_y_;
    move_base_goal_.target_pose.pose.position.z = home_robot_z_;
    move_base_goal_.target_pose.pose.orientation.x = home_robot_rot_x_;
    move_base_goal_.target_pose.pose.orientation.y = home_robot_rot_y_;
    move_base_goal_.target_pose.pose.orientation.z = home_robot_rot_z_;
    move_base_goal_.target_pose.pose.orientation.w = home_robot_rot_w_;
    move_base_ac_->sendGoal(move_base_goal_, boost::bind(&SermasPilot::moveBaseResultCb, this, _1, _2));

    navigation_command_sent_ = true;
  }
}

//! 18_CALCULATING_GOAL
// The RB-1 calculates which is the closest rack goal
void SermasPilot::calculatingGoalState()
{
  RCOMPONENT_INFO_STREAM("18_CALCULATING_GOAL");

  double distance_1 = sqrt(pow(rack_x_ - home_docking_x_, 2) + pow(rack_y_ - home_docking_y_, 2));
  double distance_2 = sqrt(pow(rack_x_ - room_1_docking_x_, 2) + pow(rack_y_ - room_1_docking_x_, 2));
  double distance_3 = sqrt(pow(rack_x_ - room_2_docking_x_, 2) + pow(rack_y_ - room_2_docking_y_, 2));

  double min_distance = distance_1;
  x_goal_ = home_pre_pick_x_;
  y_goal_ = home_pre_pick_y_;
  z_goal_ = home_pre_pick_z_;
  x_orient_goal_ = home_pre_pick_rot_x_;
  y_orient_goal_ = home_pre_pick_rot_y_;
  z_orient_goal_ = home_pre_pick_rot_z_;
  w_orient_goal_ = home_pre_pick_rot_w_;

  if (distance_2 < min_distance)
  {
    min_distance = distance_2;
    x_goal_ = home_pre_pick_x_;
    y_goal_ = home_pre_pick_y_;
    z_goal_ = home_pre_pick_z_;
    x_orient_goal_ = home_pre_pick_rot_x_;
    y_orient_goal_ = home_pre_pick_rot_y_;
    z_orient_goal_ = home_pre_pick_rot_z_;
    w_orient_goal_ = home_pre_pick_rot_w_;
  }
  if (distance_3 < min_distance)
  {
    x_goal_ = room_2_pre_pick_x_;
    y_goal_ = room_2_pre_pick_y_;
    z_goal_ = room_2_pre_pick_z_;
    x_orient_goal_ = room_2_pre_pick_rot_x_;
    y_orient_goal_ = room_2_pre_pick_rot_y_;
    z_orient_goal_ = room_2_pre_pick_rot_z_;
    w_orient_goal_ = room_2_pre_pick_rot_w_;
  }

  std_srvs::TriggerRequest goal_calculated_srv_request;
  std_srvs::TriggerResponse goal_calculated_srv_response;

  if (goalCalculatedServiceCb(goal_calculated_srv_request, goal_calculated_srv_response))
  {
    if (goal_calculated_srv_response.success)
    {
      RCOMPONENT_INFO_STREAM("Successfully switched from 18_CALCULATING_GOAL to 19_NAVIGATING_TO_RACK");
    }
    else
    {
      RCOMPONENT_WARN_STREAM("Failed to switch from 18_CALCULATING_GOAL to 19_NAVIGATING_TO_RACK: " << goal_calculated_srv_response.message.c_str());
    }
  }
  else
  {
    RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/goal_calculated");
  }
}

//! 21_CHARGING_RACK
// The RB-1 is charging the rack
void SermasPilot::chargingRackState()
{
  RCOMPONENT_INFO_STREAM("21_CHARGING_RACK");
}
/* States !*/

/*** Transitions ***/
// _Pick Up_ Mission
//! 1_WAITING_FOR_MISSION --> 2_CHECKING_ELEVATOR
bool SermasPilot::pickupMissionReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "1_WAITING_FOR_MISSION")
  {
    changeState("2_CHECKING_ELEVATOR", "_Pick Up_ Mission received!");
    response.success = true;
    response.message = "_Pick Up_ Mission received! Switching from 1_WAITING_FOR_MISSION to 2_CHECKING_ELEVATOR.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 1_WAITING_FOR_MISSION state!";
    return true;
  }
  return false;
}

//! 2_CHECKING_ELEVATOR --> 3_GETTING_RACK_POSITION, or 16_CHECKING_ELEVATOR --> 17_GETTING_RACK_POSITION
bool SermasPilot::elevatorDownServiceCb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
  if (current_state_ == "2_CHECKING_ELEVATOR")
  {
    if (request.data)
    {
      changeState("3_GETTING_RACK_POSITION", "The elevator is down!");
      response.success = true;
      response.message = "The elevator is down! Switching from 2_CHECKING_ELEVATOR to 3_GETTING_RACK_POSITION.";
      return true;
    }
    else
    {
      response.success = true;
      response.message = "The elevator is up! Waiting for the elevator to go down...";
      return true;
    }
  }
  else if (current_state_ == "16_CHECKING_ELEVATOR")
  {
    if (request.data)
    {
      changeState("17_GETTING_RACK_POSITION", "The elevator is down!");
      response.success = true;
      response.message = "The elevator is down! Switching from 16_CHECKING_ELEVATOR to 17_GETTING_RACK_POSITION.";
      return true;
    }
    else
    {
      response.success = true;
      response.message = "The elevator is up! Waiting for the elevator to go down...";
      return true;
    }
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in CHECKING_ELEVATOR (2. or 16.) state!";
    return true;
  }
  return false;
}

//! 3_GETTING_RACK_POSITION --> 4_CHECKING_RACK_POSITION, or 17_GETTING_RACK_POSITION --> 18_CALCULATING_GOAL
bool SermasPilot::rackPositionReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "3_GETTING_RACK_POSITION")
  {
    changeState("4_CHECKING_RACK_POSITION", "Rack position received from RTLS: x=" + std::to_string(rack_x_) + ", y=" + std::to_string(rack_y_));
    response.success = true;
    response.message = "Location received! Switching from 3_GETTING_RACK_POSITION to 4_CHECKING_RACK_POSITION.";
    return true;
  }
  else if (current_state_ == "17_GETTING_RACK_POSITION")
  {
    changeState("18_CALCULATING_GOAL", "Rack position received from RTLS: x=" + std::to_string(rack_x_) + ", y=" + std::to_string(rack_y_));
    response.success = true;
    response.message = "Location received! Switching from 17_GETTING_RACK_POSITION to 18_CALCULATING_GOAL.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in GETTING_RACK_POSITION (3. or 17.) state!";
    return true;
  }
  return false;
}

//! 4_CHECKING_RACK_POSITION --> 3_GETTING_RACK_POSITION or 5_NAVIGATING_TO_RACK
bool SermasPilot::correctPositionServiceCb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
  if (current_state_ == "4_CHECKING_RACK_POSITION")
  {
    if (request.data)
    {
      changeState("5_NAVIGATING_TO_RACK", "The rack is in the correct room!");
      response.success = true;
      response.message = "The rack is in the correct room! Switching from 4_CHECKING_RACK_POSITION to 5_NAVIGATING_TO_RACK.";
      return true;
    }
    else
    {
      changeState("3_GETTING_RACK_POSITION", "The rack is NOT in the correct room!");
      response.success = false;
      response.message = "The rack is NOT in the correct room! Switching from 4_CHECKING_RACK_POSITION to 3_GETTING_RACK_POSITION.";
      return true;
    }
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 4_CHECKING_RACK_POSITION state!";
    return true;
  }
  return false;
}

//! 5_NAVIGATING_TO_RACK --> 6_PICKING_RACK, or 19_NAVIGATING_TO_RACK --> 20_PICKING_RACK
bool SermasPilot::arrivedAtRackServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "5_NAVIGATING_TO_RACK")
  {
    changeState("6_PICKING_RACK", "Arrived at rack!");
    response.success = true;
    response.message = "Arrived at rack! Switching from 5_NAVIGATING_TO_RACK to 6_PICKING_RACK.";
    return true;
  }
  else if (current_state_ == "19_NAVIGATING_TO_RACK")
  {
    changeState("20_PICKING_RACK", "Arrived at rack!");
    response.success = true;
    response.message = "Arrived at rack! Switching from 19_NAVIGATING_TO_RACK to 20_PICKING_RACK.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in NAVIGATING_TO_RACK (5. or 19.) state!";
    return true;
  }
  return false;
}

//! 6_PICKING_RACK --> 7_WAITING_IN_FIRST_ROOM, or 20_PICKING_RACK --> 21_CHARGING_RACK
bool SermasPilot::rackPickedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "6_PICKING_RACK")
  {
    changeState("7_WAITING_IN_FIRST_ROOM", "Rack picked!");
    response.success = true;
    response.message = "Rack picked! Switching from 6_PICKING_RACK to 7_WAITING_IN_FIRST_ROOM.";
    return true;
  }
  else if (current_state_ == "20_PICKING_RACK")
  {
    changeState("21_CHARGING_RACK", "Rack picked!");
    response.success = true;
    response.message = "Rack picked! Switching from 20_PICKING_RACK to 21_CHARGING_RACK.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in PICKING_RACK (6. or 20.) state!";
    return true;
  }
  return false;
}

//! 7_WAITING_IN_FIRST_ROOM --> 8_NAVIGATING_TO_SECOND_ROOM
bool SermasPilot::goFromFirstToSecondRoomServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "7_WAITING_IN_FIRST_ROOM")
  {
    changeState("8_NAVIGATING_TO_SECOND_ROOM", "The 'GO TO ROOM 2' button is pressed in the HMI!");
    response.success = true;
    response.message = "The 'GO TO ROOM 2' button is pressed in the HMI! Switching from 7_WAITING_IN_FIRST_ROOM to 8_NAVIGATING_TO_SECOND_ROOM.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 7_WAITING_IN_FIRST_ROOM state!";
    return true;
  }
  return false;
}

//! 8_NAVIGATING_TO_SECOND_ROOM --> 9_WAITING_IN_SECOND_ROOM
bool SermasPilot::arrivedAtSecondRoomServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "8_NAVIGATING_TO_SECOND_ROOM")
  {
    changeState("9_WAITING_IN_SECOND_ROOM", "Arrived at second room!");
    response.success = true;
    response.message = "Arrived at second room! Switching from 8_NAVIGATING_TO_SECOND_ROOM to 9_WAITING_IN_SECOND_ROOM.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 8_NAVIGATING_TO_SECOND_ROOM state!";
    return true;
  }
  return false;
}

//! 9_WAITING_IN_SECOND_ROOM or 11_WAITING_IN_NEXT_ROOM --> 12_HOMING_RACK or 14_RELEASING_RACK
bool SermasPilot::releaseRackServiceCb(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
  if (current_state_ == "9_WAITING_IN_SECOND_ROOM")
  {
    if (request.data)
    {
      changeState("14_RELEASING_RACK", "'RELEASE AND HOME' button was pressed!");
      response.success = true;
      response.message = "'RELEASE AND HOME' button was pressed! Switching from 9_WAITING_IN_SECOND_ROOM to 14_RELEASING_RACK.";
      return true;
    }
    else
    {
      changeState("12_HOMING_RACK", "'BRING RACK HOME' button was pressed!");
      response.success = true;
      response.message = "'BRING RACK HOME' button was pressed! Switching from 9_WAITING_IN_SECOND_ROOM to 12_HOMING_RACK.";
      return true;
    }
  }
  else if (current_state_ == "11_WAITING_IN_NEXT_ROOM")
  {
    if (request.data)
    {
      changeState("14_RELEASING_RACK", "'RELEASE AND HOME' button was pressed!");
      response.success = true;
      response.message = "'RELEASE AND HOME' button was pressed! Switching from 11_WAITING_IN_NEXT_ROOM to 14_RELEASING_RACK.";
      return true;
    }
    else
    {
      changeState("12_HOMING_RACK", "'BRING RACK HOME' button was pressed!");
      response.success = true;
      response.message = "'BRING RACK HOME' button was pressed! Switching from 11_WAITING_IN_NEXT_ROOM to 12_HOMING_RACK.";
      return true;
    }
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 9_WAITING_IN_SECOND_ROOM or 11_WAITING_IN_NEXT_ROOM state!";
    return true;
  }
  return false;
}

//! 9_WAITING_IN_SECOND_ROOM --> 10_NAVIGATING_TO_NEXT_ROOM
bool SermasPilot::goFromSecondToNextRoomServiceCb(odin_msgs::StringTrigger::Request &request, odin_msgs::StringTrigger::Response &response)
{
  if (current_state_ == "9_WAITING_IN_SECOND_ROOM")
  {
    changeState("10_NAVIGATING_TO_NEXT_ROOM", "The '" + request.input + "' button is pressed in the HMI!");
    response.success = true;
    response.message = "The '" + request.input + "' button is pressed in the HMI!! Switching from 9_WAITING_IN_SECOND_ROOM to 10_NAVIGATING_TO_NEXT_ROOM.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 9_WAITING_IN_SECOND_ROOM state!";
    return true;
  }
  return false;
}

//! 10_NAVIGATING_TO_NEXT_ROOM --> 11_WAITING_IN_NEXT_ROOM
bool SermasPilot::arrivedAtNextRoomServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "10_NAVIGATING_TO_NEXT_ROOM")
  {
    changeState("11_WAITING_IN_NEXT_ROOM", "Arrived at next room!");
    response.success = true;
    response.message = "Arrived at next room! Switching from 10_NAVIGATING_TO_NEXT_ROOM to 11_WAITING_IN_NEXT_ROOM.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 10_NAVIGATING_TO_NEXT_ROOM state!";
    return true;
  }
  return false;
}

//! 11_WAITING_IN_NEXT_ROOM --> 10_NAVIGATING_TO_NEXT_ROOM
bool SermasPilot::goToNextRoomServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "11_WAITING_IN_NEXT_ROOM")
  {
    changeState("10_NAVIGATING_TO_NEXT_ROOM", "Rack picked!");
    response.success = true;
    response.message = "Rack picked! Switching from 11_WAITING_IN_NEXT_ROOM to 10_NAVIGATING_TO_NEXT_ROOM.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 11_WAITING_IN_NEXT_ROOM state!";
    return true;
  }
  return false;
}

//! 14_RELEASING_RACK --> 15_NAVIGATING_TO_HOME, or 22_RELEASING_RACK to 23_NAVIGATING_TO_HOME
bool SermasPilot::rackReleasedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "14_RELEASING_RACK")
  {
    changeState("15_NAVIGATING_TO_HOME", "Rack released!");
    response.success = true;
    response.message = "Rack released! Switching from 14_RELEASING_RACK to 15_NAVIGATING_TO_HOME.";
    return true;
  }
  else if (current_state_ == "22_RELEASING_RACK")
  {
    changeState("23_NAVIGATING_TO_HOME", "Rack released!");
    response.success = true;
    response.message = "Rack released! Switching from 22_RELEASING_RACK to 23_NAVIGATING_TO_HOME.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in NAVIGATING_TO_HOME (15. or 22.) state!";
    return true;
  }
  return false;
}

//! 12_HOMING_RACK --> 13_PLACING_RACK
bool SermasPilot::rackHomedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "12_HOMING_RACK")
  {
    changeState("13_PLACING_RACK", "Rack homed!");
    response.success = true;
    response.message = "Rack homed! Switching from 12_HOMING_RACK to 13_PLACING_RACK.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 12_HOMING_RACK state!";
    return true;
  }
  return false;
}

//! 13_PLACING_RACK --> 15_NAVIGATING_TO_HOME
bool SermasPilot::rackPlacedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "13_PLACING_RACK")
  {
    changeState("15_NAVIGATING_TO_HOME", "Rack placed!");
    response.success = true;
    response.message = "Rack placed! Switching from 13_PLACING_RACK to 15_NAVIGATING_TO_HOME.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 13_PLACING_RACK state!";
    return true;
  }
  return false;
}

//! 15_NAVIGATING_TO_HOME or 23_NAVIGATING_TO_HOME --> 1_WAITING_FOR_MISSION
bool SermasPilot::arrivedAtHomeServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "15_NAVIGATING_TO_HOME")
  {
    changeState("1_WAITING_FOR_MISSION", "Arrived at home!");
    response.success = true;
    response.message = "Arrived at home! Switching from 15_NAVIGATING_TO_HOME to 1_WAITING_FOR_MISSION.";
    return true;
  }
  else if (current_state_ == "23_NAVIGATING_TO_HOME")
  {
    changeState("1_WAITING_FOR_MISSION", "Arrived at home!");
    response.success = true;
    response.message = "Arrived at home! Switching from 23_NAVIGATING_TO_HOME to 1_WAITING_FOR_MISSION.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in NAVIGATING_TO_HOME (15. or 23.) state!";
    return true;
  }
  return false;
}

// _Recharge_ Mission
//! 1_WAITING_FOR_MISSION --> 16_CHECKING_ELEVATOR
bool SermasPilot::rechargeMissionReceivedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "1_WAITING_FOR_MISSION")
  {
    changeState("16_CHECKING_ELEVATOR", "Goal calculated!");
    response.success = true;
    response.message = "Goal calculated! Switching from 1_WAITING_FOR_MISSION to 16_CHECKING_ELEVATOR.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 1_WAITING_FOR_MISSION state!";
    return true;
  }
  return false;
}

//! 18_CALCULATING_GOAL --> 19_NAVIGATING_TO_RACK
bool SermasPilot::goalCalculatedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "18_CALCULATING_GOAL")
  {
    changeState("19_NAVIGATING_TO_RACK", "Goal calculated!");
    response.success = true;
    response.message = "Goal calculated! Switching from 18_CALCULATING_GOAL to 19_NAVIGATING_TO_RACK.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 18_CALCULATING_GOAL state!";
    return true;
  }
  return false;
}

//! 21_CHARGING_RACK --> 22_RELEASING_RACK
bool SermasPilot::rackChargedServiceCb(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
  if (current_state_ == "21_CHARGING_RACK")
  {
    changeState("22_RELEASING_RACK", "Goal calculated!");
    response.success = true;
    response.message = "Goal calculated! Switching from 21_CHARGING_RACK to 22_RELEASING_RACK.";
    return true;
  }
  else
  {
    response.success = false;
    response.message = "Service called in inappropriate state, currently not in 21_CHARGING_RACK state!";
    return true;
  }
  return false;
}
/* Transitions !*/

/* Callbacks */
//! Subscription Callbacks
// 1_WAITING_FOR_MISSION --> 16_CHECKING_ELEVATOR
void SermasPilot::smartboxSubCb(const odin_msgs::SmartboxStatus::ConstPtr &msg)
{
  if (current_state_ == "1_WAITING_FOR_MISSION")
  {
    float battery = msg->data.battery;

    if (battery < 10.0)
    {
      std_srvs::TriggerRequest recharge_mission_received_srv_request;
      std_srvs::TriggerResponse recharge_mission_received_srv_response;

      if (rechargeMissionReceivedServiceCb(recharge_mission_received_srv_request, recharge_mission_received_srv_response))
      {
        if (recharge_mission_received_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 1_WAITING_FOR_MISSION to 16_CHECKING_ELEVATOR");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 1_WAITING_FOR_MISSION to 16_CHECKING_ELEVATOR: " << recharge_mission_received_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/recharge_mission_received");
      }
    }
  }
  else if (current_state_ == "21_CHARGING_RACK")
  {
    float battery = msg->data.battery;

    if (battery > 90.0)
    {
      std_srvs::TriggerRequest rack_charged_srv_request;
      std_srvs::TriggerResponse rack_charged_srv_response;

      if (rackChargedServiceCb(rack_charged_srv_request, rack_charged_srv_response))
      {
        if (rack_charged_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 21_CHARGING_RACK to 22_RELEASING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 21_CHARGING_RACK to 22_RELEASING_RACK: " << rack_charged_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/rack_charged");
      }
    }
  }
  tickTopicsHealth(smartbox_sub_name_);
}

// 2_CHECKING_ELEVATOR --> 3_GETTING_RACK_POSITION, or 16_CHECKING_ELEVATOR --> 17_GETTING_RACK_POSITION
void SermasPilot::elevatorSubCb(const robotnik_msgs::ElevatorStatus::ConstPtr &msg)
{
  if (current_state_ == "2_CHECKING_ELEVATOR")
  {
    std::string message = msg->position;
    RCOMPONENT_WARN_STREAM("Received message from Elevator: " + message);

    if (message == "down")
    {
      std_srvs::SetBoolRequest elevator_down_srv_request;
      std_srvs::SetBoolResponse elevator_down_srv_response;

      elevator_down_srv_request.data = true;

      if (elevatorDownServiceCb(elevator_down_srv_request, elevator_down_srv_response))
      {
        if (elevator_down_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 2_CHECKING_ELEVATOR to 3_GETTING_RACK_POSITION");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 2_CHECKING_ELEVATOR to 3_GETTING_RACK_POSITION: " << elevator_down_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/elevator_down");
      }
    }
    else if (message == "up")
    {
      RCOMPONENT_ERROR_STREAM("Elevator is up, waiting for it to go down...");
      // TODO: set the elevator down?
    }
  }
  else if (current_state_ == "16_CHECKING_ELEVATOR")
  {
    std::string message = msg->position;
    RCOMPONENT_WARN_STREAM("Received message from Elevator: " + message);

    if (message == "down")
    {
      std_srvs::SetBoolRequest elevator_down_srv_request;
      std_srvs::SetBoolResponse elevator_down_srv_response;

      elevator_down_srv_request.data = true;

      if (elevatorDownServiceCb(elevator_down_srv_request, elevator_down_srv_response))
      {
        if (elevator_down_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 16_CHECKING_ELEVATOR to 17_GETTING_RACK_POSITION");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 16_CHECKING_ELEVATOR to 17_GETTING_RACK_POSITION: " << elevator_down_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/elevator_down");
      }
    }
    else if (message == "up")
    {
      RCOMPONENT_ERROR_STREAM("Elevator is up, waiting for it to go down...");
      // TODO: set the elevator down
    }
  }
  tickTopicsHealth(elevator_sub_name_);
}

// 3_GETTING_RACK_POSITION --> 4_CHECKING_RACK_POSITION, or 17_GETTING_RACK_POSITION --> 18_CALCULATING_GOAL
void SermasPilot::rtlsSubCb(const odin_msgs::RTLSBase::ConstPtr &msg)
{
  if (current_state_ == "3_GETTING_RACK_POSITION")
  {
    // TODO during pilot: Set correct ID
    if (msg->data.id == "ble-pd-601283DE0245" or msg->data.id == "ble-pd-601283DE0246")
    {
      rack_x_ = msg->data.data.x;
      rack_y_ = msg->data.data.y;
      rack_z_ = msg->data.data.z;

      std_srvs::TriggerRequest rack_position_received_srv_request;
      std_srvs::TriggerResponse rack_position_received_srv_response;

      if (rackPositionReceivedServiceCb(rack_position_received_srv_request, rack_position_received_srv_response))
      {
        if (rack_position_received_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 3_GETTING_RACK_POSITION to 4_CHECKING_RACK_POSITION");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 3_GETTING_RACK_POSITION to 4_CHECKING_RACK_POSITION: " << rack_position_received_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/rack_position_received");
      }
    }
  }
  else if (current_state_ == "17_GETTING_RACK_POSITION")
  {
    // TODO during pilot: Set correct IDs and parametrize them
    if (msg->data.id == "ble-pd-601283DE0245" or msg->data.id == "ble-pd-601283DE0246")
    {
      rack_x_ = msg->data.data.x;
      rack_y_ = msg->data.data.y;
      rack_z_ = msg->data.data.z;

      std_srvs::TriggerRequest rack_position_received_srv_request;
      std_srvs::TriggerResponse rack_position_received_srv_response;

      if (rackPositionReceivedServiceCb(rack_position_received_srv_request, rack_position_received_srv_response))
      {
        if (rack_position_received_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 17_GETTING_RACK_POSITION to 18_CALCULATING_GOAL");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 17_GETTING_RACK_POSITION to 18_CALCULATING_GOAL: " << rack_position_received_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/rack_position_received");
      }
    }
  }
  tickTopicsHealth(rtls_sub_name_);
}

void SermasPilot::hmiSubCb(const odin_msgs::HMIBase::ConstPtr &msg)
{
  // 1_WAITING_FOR_MISSION --> 2_CHECKING_ELEVATOR
  if (current_state_ == "1_WAITING_FOR_MISSION")
  {
    std::string message = msg->data.data.taskType;
    RCOMPONENT_WARN_STREAM("Received message from HMI: " + message);

    if (message == "PICK UP RACK")
    {
      std_srvs::TriggerRequest pickup_mission_received_srv_request;
      std_srvs::TriggerResponse pickup_mission_received_srv_response;

      if (pickupMissionReceivedServiceCb(pickup_mission_received_srv_request, pickup_mission_received_srv_response))
      {
        if (pickup_mission_received_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 1_WAITING_FOR_MISSION to 2_CHECKING_ELEVATOR");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 1_WAITING_FOR_MISSION to 2_CHECKING_ELEVATOR: " << pickup_mission_received_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/pickup_mission_received");
      }
    }
  }

  // 7_WAITING_IN_FIRST_ROOM --> 8_NAVIGATING_TO_SECOND_ROOM
  else if (current_state_ == "7_WAITING_IN_FIRST_ROOM")
  {
    std::string message = msg->data.data.taskType;
    RCOMPONENT_WARN_STREAM("Received message from HMI: " + message);

    if (message == "GO TO ROOM 2")
    {
      // When waiting in the first room, we can only go to the second room, so we don't need the coordinates
      // if (msg->data.data.endLocation.position.size() > 1 && msg->data.data.endLocation.orientation.size() > 1)
      // {
      //   next_room_x_ = msg->data.data.endLocation.position[0];
      //   next_room_y_ = msg->data.data.endLocation.position[1];
      //   next_room_z_ = msg->data.data.endLocation.position[2];
      //   next_room_rot_x_ = msg->data.data.endLocation.orientation[0];
      //   next_room_rot_y_ = msg->data.data.endLocation.orientation[1];
      //   next_room_rot_z_ = msg->data.data.endLocation.orientation[2];
      //   next_room_rot_w_ = msg->data.data.endLocation.orientation[3];
      // }
      // else
      // {
      //   RCOMPONENT_ERROR_STREAM("Invalid position and orientation data");
      //   return;
      // }

      std_srvs::TriggerRequest go_from_first_to_second_room_srv_request;
      std_srvs::TriggerResponse go_from_first_to_second_room_srv_response;

      if (goFromFirstToSecondRoomServiceCb(go_from_first_to_second_room_srv_request, go_from_first_to_second_room_srv_response))
      {
        if (go_from_first_to_second_room_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 7_WAITING_IN_FIRST_ROOM to 8_NAVIGATING_TO_SECOND_ROOM");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 7_WAITING_IN_FIRST_ROOM to 8_NAVIGATING_TO_SECOND_ROOM: " << go_from_first_to_second_room_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/go_from_first_to_second_room");
      }
    }
  }

  // 9_WAITING_IN_SECOND_ROOM --> 10_NAVIGATING_TO_NEXT_ROOM, 12_HOMING_RACK, or 14_RELEASING_RACK
  else if (current_state_ == "9_WAITING_IN_SECOND_ROOM")
  {
    std::string message = msg->data.data.taskType;
    RCOMPONENT_WARN_STREAM("Received message from HMI: " + message);

    // 9_WAITING_IN_SECOND_ROOM --> 10_NAVIGATING_TO_NEXT_ROOM
    if (message == "GO TO ROOM 1" || message == "GO TO ROOM 3")
    {
      if (msg->data.data.endLocation.position.size() > 1 && msg->data.data.endLocation.orientation.size() > 1)
      {
        next_room_x_ = msg->data.data.endLocation.position[0];
        next_room_y_ = msg->data.data.endLocation.position[1];
        next_room_z_ = msg->data.data.endLocation.position[2];
        next_room_rot_x_ = msg->data.data.endLocation.orientation[0];
        next_room_rot_y_ = msg->data.data.endLocation.orientation[1];
        next_room_rot_z_ = msg->data.data.endLocation.orientation[2];
        next_room_rot_w_ = msg->data.data.endLocation.orientation[3];
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Invalid position and orientation data");
        return;
      }

      odin_msgs::StringTriggerRequest go_from_second_to_next_srv_request;
      go_from_second_to_next_srv_request.input = message;
      odin_msgs::StringTriggerResponse go_from_second_to_next_srv_response;

      if (goFromSecondToNextRoomServiceCb(go_from_second_to_next_srv_request, go_from_second_to_next_srv_response))
      {
        if (go_from_second_to_next_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 9_WAITING_IN_SECOND_ROOM to 10_NAVIGATING_TO_NEXT_ROOM");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 9_WAITING_IN_SECOND_ROOM to 10_NAVIGATING_TO_NEXT_ROOM: " << go_from_second_to_next_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/go_from_second_to_next_room");
      }
    }

    // 9_WAITING_IN_SECOND_ROOM --> 12_HOMING_RACK
    else if (message == "BRING RACK HOME")
    {
      std_srvs::SetBoolRequest release_and_home_srv_request;
      std_srvs::SetBoolResponse release_and_home_srv_response;

      release_and_home_srv_request.data = false;

      if (releaseRackServiceCb(release_and_home_srv_request, release_and_home_srv_response))
      {
        if (release_and_home_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 9_WAITING_IN_SECOND_ROOM to 12_HOMING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 9_WAITING_IN_SECOND_ROOM to 12_HOMING_RACK: " << release_and_home_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/release_rack");
      }
    }

    // 9_WAITING_IN_SECOND_ROOM --> 14_RELEASING_RACK
    else if (message == "RELEASE AND HOME")
    {
      std_srvs::SetBoolRequest release_and_home_srv_request;
      std_srvs::SetBoolResponse release_and_home_srv_response;

      release_and_home_srv_request.data = true;

      if (releaseRackServiceCb(release_and_home_srv_request, release_and_home_srv_response))
      {
        if (release_and_home_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 9_WAITING_IN_SECOND_ROOM to 14_RELEASING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 9_WAITING_IN_SECOND_ROOM to 14_RELEASING_RACK: " << release_and_home_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/release_rack");
      }
    }
  }

  // 11_WAITING_IN_NEXT_ROOM --> 10_NAVIGATING_TO_NEXT_ROOM, 12_HOMING_RACK, or 14_RELEASING_RACK
  else if (current_state_ == "11_WAITING_IN_NEXT_ROOM")
  {
    std::string message = msg->data.data.taskType;
    RCOMPONENT_WARN_STREAM("Received message from HMI: " + message);

    // 11_WAITING_IN_NEXT_ROOM --> 10_NAVIGATING_TO_NEXT_ROOM
    // TODO: Add all possible rooms?
    if (message == "GO TO ROOM X")
    {
      std_srvs::TriggerRequest go_to_next_room_srv_request;
      std_srvs::TriggerResponse go_to_next_room_srv_response;

      if (goToNextRoomServiceCb(go_to_next_room_srv_request, go_to_next_room_srv_response))
      {
        if (go_to_next_room_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 11_WAITING_IN_NEXT_ROOM to 10_NAVIGATING_TO_NEXT_ROOM");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 11_WAITING_IN_NEXT_ROOM to 10_NAVIGATING_TO_NEXT_ROOM: " << go_to_next_room_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/go_to_next_room");
      }
    }

    // 11_WAITING_IN_NEXT_ROOM --> 12_HOMING_RACK
    else if (message == "BRING RACK HOME")
    {
      std_srvs::SetBoolRequest release_and_home_srv_request;
      std_srvs::SetBoolResponse release_and_home_srv_response;

      release_and_home_srv_request.data = false;

      if (releaseRackServiceCb(release_and_home_srv_request, release_and_home_srv_response))
      {
        if (release_and_home_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 11_WAITING_IN_NEXT_ROOM to 12_HOMING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 11_WAITING_IN_NEXT_ROOM to 12_HOMING_RACK: " << release_and_home_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/release_rack");
      }
    }

    // 11_WAITING_IN_NEXT_ROOM --> 14_RELEASING_RACK
    else if (message == "RELEASE AND HOME")
    {
      std_srvs::SetBoolRequest release_and_home_srv_request;
      std_srvs::SetBoolResponse release_and_home_srv_response;

      release_and_home_srv_request.data = true;

      if (releaseRackServiceCb(release_and_home_srv_request, release_and_home_srv_response))
      {
        if (release_and_home_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 11_WAITING_IN_NEXT_ROOM to 14_RELEASING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 11_WAITING_IN_NEXT_ROOM to 14_RELEASING_RACK: " << release_and_home_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/release_rack");
      }
    }
  }
  tickTopicsHealth(hmi_sub_name_);
}

void SermasPilot::batterySubCb(const robotnik_msgs::BatteryStatus::ConstPtr &msg)
{
  robot_status_.data.battery = battery_status_;
  battery_status_ = msg->level;
  tickTopicsHealth(battery_sub_name_);
}

void SermasPilot::poseSubCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  robot_status_.data.pose = pose_;
  pose_ = *msg;
  tickTopicsHealth(pose_sub_name_);
}

//! Action Callbacks
void SermasPilot::moveBaseResultCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std_srvs::TriggerRequest move_base_srv_request;
    std_srvs::TriggerResponse move_base_srv_response;

    // 5_NAVIGATING_TO_RACK --> 6_PICKING_RACK
    if (current_state_ == "5_NAVIGATING_TO_RACK")
    {
      if (arrivedAtRackServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 5_NAVIGATING_TO_RACK to 6_PICKING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 5_NAVIGATING_TO_RACK to 6_PICKING_RACK: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/arrived_at_rack");
      }
    }

    // 19_NAVIGATING_TO_RACK --> 20_PICKING_RACK
    if (current_state_ == "19_NAVIGATING_TO_RACK")
    {
      if (arrivedAtRackServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 19_NAVIGATING_TO_RACK to 20_PICKING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 19_NAVIGATING_TO_RACK to 20_PICKING_RACK: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/arrived_at_rack");
      }
    }

    // 8_NAVIGATING_TO_SECOND_ROOM --> 9_WAITING_IN_SECOND_ROOM
    if (current_state_ == "8_NAVIGATING_TO_SECOND_ROOM")
    {
      if (arrivedAtSecondRoomServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 8_NAVIGATING_TO_SECOND_ROOM to 9_WAITING_IN_SECOND_ROOM");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 8_NAVIGATING_TO_SECOND_ROOM to 9_WAITING_IN_SECOND_ROOM: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/arrived_at_second_room");
      }
    }

    // 10_NAVIGATING_TO_NEXT_ROOM --> 11_WAITING_IN_NEXT_ROOM
    if (current_state_ == "10_NAVIGATING_TO_NEXT_ROOM")
    {
      if (arrivedAtNextRoomServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 10_NAVIGATING_TO_NEXT_ROOM to 11_WAITING_IN_NEXT_ROOM");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 10_NAVIGATING_TO_NEXT_ROOM to 11_WAITING_IN_NEXT_ROOM: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/arrived_at_next_room");
      }
    }

    // 12_HOMING_RACK --> 13_PLACING_RACK
    if (current_state_ == "12_HOMING_RACK")
    {
      if (rackHomedServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 12_HOMING_RACK to 13_PLACING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 12_HOMING_RACK to 13_PLACING_RACK: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/rack_homed");
      }
    }

    // 15_NAVIGATING_TO_HOME --> 1_WAITING_FOR_MISSION
    if (current_state_ == "15_NAVIGATING_TO_HOME")
    {
      if (arrivedAtHomeServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 15_NAVIGATING_TO_HOME to 1_WAITING_FOR_MISSION");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 15_NAVIGATING_TO_HOME to 1_WAITING_FOR_MISSION: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/arrived_at_home");
      }
    }

    // 23_NAVIGATING_TO_HOME --> 1_WAITING_FOR_MISSION
    if (current_state_ == "23_NAVIGATING_TO_HOME")
    {
      if (arrivedAtHomeServiceCb(move_base_srv_request, move_base_srv_response))
      {
        if (move_base_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 23_NAVIGATING_TO_HOME to 1_WAITING_FOR_MISSION");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 23_NAVIGATING_TO_HOME to 1_WAITING_FOR_MISSION: " << move_base_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/arrived_at_home");
      }
    }
  }
}

void SermasPilot::commandSequencerResultCb(const actionlib::SimpleClientGoalState &state, const robot_simple_command_manager_msgs::RobotSimpleCommandResultConstPtr &result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std_srvs::TriggerRequest command_sequencer_srv_request;
    std_srvs::TriggerResponse command_sequencer_srv_response;

    // 6_PICKING_RACK --> 7_WAITING_IN_FIRST_ROOM
    if (current_state_ == "6_PICKING_RACK")
    {
      if (rackPickedServiceCb(command_sequencer_srv_request, command_sequencer_srv_response))
      {
        if (command_sequencer_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 6_PICKING_RACK to 7_WAITING_IN_FIRST_ROOM");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 6_PICKING_RACK to 7_WAITING_IN_FIRST_ROOM: " << command_sequencer_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/rack_picked");
      }
    }

    // 20_PICKING_RACK --> 21_CHARGING_RACK
    else if (current_state_ == "20_PICKING_RACK")
    {
      if (rackPickedServiceCb(command_sequencer_srv_request, command_sequencer_srv_response))
      {
        if (command_sequencer_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 20_PICKING_RACK to 21_CHARGING_RACK");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 20_PICKING_RACK to 21_CHARGING_RACK: " << command_sequencer_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/rack_picked");
      }
    }

    // 13_PLACING_RACK --> 15_NAVIGATING_TO_HOME
    else if (current_state_ == "13_PLACING_RACK")
    {
      if (rackPlacedServiceCb(command_sequencer_srv_request, command_sequencer_srv_response))
      {
        if (command_sequencer_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 13_PLACING_RACK to 15_NAVIGATING_TO_HOME");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 13_PLACING_RACK to 15_NAVIGATING_TO_HOME: " << command_sequencer_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/rack_placed");
      }
    }

    // 14_RELEASING_RACK --> 15_NAVIGATING_TO_HOME
    else if (current_state_ == "14_RELEASING_RACK")
    {
      if (rackReleasedServiceCb(command_sequencer_srv_request, command_sequencer_srv_response))
      {
        if (command_sequencer_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 14_RELEASING_RACK to 15_NAVIGATING_TO_HOME");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 14_RELEASING_RACK to 15_NAVIGATING_TO_HOME: " << command_sequencer_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/rack_released");
      }
    }

    // 22_RELEASING_RACK --> 23_NAVIGATING_TO_HOME
    else if (current_state_ == "22_RELEASING_RACK")
    {
      if (rackReleasedServiceCb(command_sequencer_srv_request, command_sequencer_srv_response))
      {
        if (command_sequencer_srv_response.success)
        {
          RCOMPONENT_INFO_STREAM("Successfully switched from 22_RELEASING_RACK to 23_NAVIGATING_TO_HOME");
        }
        else
        {
          RCOMPONENT_WARN_STREAM("Failed to switch from 22_RELEASING_RACK to 23_NAVIGATING_TO_HOME: " << command_sequencer_srv_response.message.c_str());
        }
      }
      else
      {
        RCOMPONENT_ERROR_STREAM("Failed to call service /sermas_pilot/rack_released");
      }
    }
  }
}

void SermasPilot::loadLocationParameters(XmlRpc::XmlRpcValue &locations)
{
  auto extractPose = [](const XmlRpc::XmlRpcValue &location, double &x, double &y, double &z,
                        double &rot_x, double &rot_y, double &rot_z, double &rot_w)
  {
    if (location.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      if (location.hasMember("x"))
        x = static_cast<double>(location["x"]);
      if (location.hasMember("y"))
        y = static_cast<double>(location["y"]);
      if (location.hasMember("z"))
        z = static_cast<double>(location["z"]);
      if (location.hasMember("rot_x"))
        rot_x = static_cast<double>(location["rot_x"]);
      if (location.hasMember("rot_y"))
        rot_y = static_cast<double>(location["rot_y"]);
      if (location.hasMember("rot_z"))
        rot_z = static_cast<double>(location["rot_z"]);
      if (location.hasMember("rot_w"))
        rot_w = static_cast<double>(location["rot_w"]);
    }
  };

  if (locations.hasMember("home"))
  {
    XmlRpc::XmlRpcValue home = locations["home"];
    if (home.hasMember("robot"))
    {
      extractPose(home["robot"], home_robot_x_, home_robot_y_, home_robot_z_, home_robot_rot_x_, home_robot_rot_y_, home_robot_rot_z_, home_robot_rot_w_);
      RCOMPONENT_INFO_STREAM("HOME ROOM robot: x=" << home_robot_x_ << ", y=" << home_robot_y_ << ", z=" << home_robot_z_ << ", rot_x=" << home_robot_rot_x_ << ", rot_y=" << home_robot_rot_y_ << ", rot_z=" << home_robot_rot_z_ << ", rot_w=" << home_robot_rot_w_);
    }
    if (home.hasMember("pre_pick"))
    {
      extractPose(home["pre_pick"], home_pre_pick_x_, home_pre_pick_y_, home_pre_pick_z_, home_pre_pick_rot_x_, home_pre_pick_rot_y_, home_pre_pick_rot_z_, home_pre_pick_rot_w_);
      RCOMPONENT_INFO_STREAM("HOME ROOM pre_pick: x=" << home_pre_pick_x_ << ", y=" << home_pre_pick_y_ << ", z=" << home_pre_pick_z_ << ", rot_x=" << home_pre_pick_rot_x_ << ", rot_y=" << home_pre_pick_rot_y_ << ", rot_z=" << home_pre_pick_rot_z_ << ", rot_w=" << home_pre_pick_rot_w_);
    }
    if (home.hasMember("place"))
    {
      extractPose(home["place"], home_place_x_, home_place_y_, home_place_z_, home_place_rot_x_, home_place_rot_y_, home_place_rot_z_, home_place_rot_w_);
      RCOMPONENT_INFO_STREAM("HOME ROOM place: x=" << home_place_x_ << ", y=" << home_place_y_ << ", z=" << home_place_z_ << ", rot_x=" << home_place_rot_x_ << ", rot_y=" << home_place_rot_y_ << ", rot_z=" << home_place_rot_z_ << ", rot_w=" << home_place_rot_w_);
    }
  }

  if (locations.hasMember("room_1"))
  {
    XmlRpc::XmlRpcValue room_1 = locations["room_1"];
    if (room_1.hasMember("pre_pick"))
    {
      extractPose(room_1["pre_pick"], room_1_pre_pick_x_, room_1_pre_pick_y_, room_1_pre_pick_z_, room_1_pre_pick_rot_x_, room_1_pre_pick_rot_y_, room_1_pre_pick_rot_z_, room_1_pre_pick_rot_w_);
      RCOMPONENT_INFO_STREAM("ROOM 1 pre_pick: x=" << room_1_pre_pick_x_ << ", y=" << room_1_pre_pick_y_ << ", z=" << room_1_pre_pick_z_ << ", rot_x=" << room_1_pre_pick_rot_x_ << ", rot_y=" << room_1_pre_pick_rot_y_ << ", rot_z=" << room_1_pre_pick_rot_z_ << ", rot_w=" << room_1_pre_pick_rot_w_);
    }
    if (room_1.hasMember("place"))
    {
      extractPose(room_1["place"], room_1_place_x_, room_1_place_y_, room_1_place_z_, room_1_place_rot_x_, room_1_place_rot_y_, room_1_place_rot_z_, room_1_place_rot_w_);
      RCOMPONENT_INFO_STREAM("ROOM 1 place: x=" << room_1_place_x_ << ", y=" << room_1_place_y_ << ", z=" << room_1_place_z_ << ", rot_x=" << room_1_place_rot_x_ << ", rot_y=" << room_1_place_rot_y_ << ", rot_z=" << room_1_place_rot_z_ << ", rot_w=" << room_1_place_rot_w_);
    }
  }

  if (locations.hasMember("room_2"))
  {
    XmlRpc::XmlRpcValue room_2 = locations["room_2"];
    if (room_2.hasMember("pre_pick"))
    {
      extractPose(room_2["pre_pick"], room_2_pre_pick_x_, room_2_pre_pick_y_, room_2_pre_pick_z_, room_2_pre_pick_rot_x_, room_2_pre_pick_rot_y_, room_2_pre_pick_rot_z_, room_2_pre_pick_rot_w_);
      RCOMPONENT_INFO_STREAM("ROOM 2 pre_pick: x=" << room_2_pre_pick_x_ << ", y=" << room_2_pre_pick_y_ << ", z=" << room_2_pre_pick_z_ << ", rot_x=" << room_2_pre_pick_rot_x_ << ", rot_y=" << room_2_pre_pick_rot_y_ << ", rot_z=" << room_2_pre_pick_rot_z_ << ", rot_w=" << room_2_pre_pick_rot_w_);
    }
    if (room_2.hasMember("place"))
    {
      extractPose(room_2["place"], room_2_place_x_, room_2_place_y_, room_2_place_z_, room_2_place_rot_x_, room_2_place_rot_y_, room_2_place_rot_z_, room_2_place_rot_w_);
      RCOMPONENT_INFO_STREAM("ROOM 2 place: x=" << room_2_place_x_ << ", y=" << room_2_place_y_ << ", z=" << room_2_place_z_ << ", rot_x=" << room_2_place_rot_x_ << ", rot_y=" << room_2_place_rot_y_ << ", rot_z=" << room_2_place_rot_z_ << ", rot_w=" << room_2_place_rot_w_);
    }
  }

  if (locations.hasMember("room_3"))
  {
    XmlRpc::XmlRpcValue room_3 = locations["room_3"];
    if (room_3.hasMember("place"))
    {
      extractPose(room_3["place"], room_3_place_x_, room_3_place_y_, room_3_place_z_, room_3_place_rot_x_, room_3_place_rot_y_, room_3_place_rot_z_, room_3_place_rot_w_);
      RCOMPONENT_INFO_STREAM("ROOM 3 place: x=" << room_3_place_x_ << ", y=" << room_3_place_y_ << ", z=" << room_3_place_z_ << ", rot_x=" << room_3_place_rot_x_ << ", rot_y=" << room_3_place_rot_y_ << ", rot_z=" << room_3_place_rot_z_ << ", rot_w=" << room_3_place_rot_w_);
    }
  }

  if (locations.hasMember("docking_stations"))
  {
    XmlRpc::XmlRpcValue docking = locations["docking_stations"];
    if (docking.hasMember("home"))
    {
      XmlRpc::XmlRpcValue homeDocking = docking["home"];
      if (homeDocking.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        if (homeDocking.hasMember("x"))
        {
          home_docking_x_ = static_cast<double>(homeDocking["x"]);
        }
        if (homeDocking.hasMember("y"))
        {
          home_docking_y_ = static_cast<double>(homeDocking["y"]);
        }
        RCOMPONENT_INFO_STREAM("HOME ROOM docking: x=" << home_docking_x_ << ", y=" << home_docking_y_);
      }
    }

    if (docking.hasMember("room_1"))
    {
      XmlRpc::XmlRpcValue room1Docking = docking["room_1"];
      if (room1Docking.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        if (room1Docking.hasMember("x"))
        {
          room_1_docking_x_ = static_cast<double>(room1Docking["x"]);
        }
        if (room1Docking.hasMember("y"))
        {
          room_1_docking_y_ = static_cast<double>(room1Docking["y"]);
        }
        RCOMPONENT_INFO_STREAM("ROOM 1 docking: x=" << room_1_docking_x_ << ", y=" << room_1_docking_y_);
      }
    }

    if (docking.hasMember("room_2"))
    {
      XmlRpc::XmlRpcValue room2Docking = docking["room_2"];
      if (room2Docking.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        if (room2Docking.hasMember("x"))
        {
          room_2_docking_x_ = static_cast<double>(room2Docking["x"]);
        }
        if (room2Docking.hasMember("y"))
        {
          room_2_docking_y_ = static_cast<double>(room2Docking["y"]);
        }
        RCOMPONENT_INFO_STREAM("ROOM 2 docking: x=" << room_2_docking_x_ << ", y=" << room_2_docking_y_);
      }
    }
  }
}

void SermasPilot::loadRtlsIds(XmlRpc::XmlRpcValue &rtlsIds)
{
  auto extractId = [&](const std::string &key, std::string &storage)
  {
    if (rtlsIds.hasMember(key))
    {
      storage = static_cast<std::string>(rtlsIds[key]);
      RCOMPONENT_INFO_STREAM(key << ": " << storage);
    }
  };

  extractId("id_1", id_1_);
  extractId("id_2", id_2_);
}
/* Callbacks !*/
