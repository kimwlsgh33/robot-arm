#include "open_manipulator_teleop/open_manipulator_teleop_keyboard.h"

/*
 * OMT (OpenManipulatorTeleop) 객체가 생성되면 실행되는 생성자
 * node_handle_, priv_node_handle_ 멤버객체를 초기화한다
 * 이 두 멤버 객체는, ROS의 Node를 Network와 통신하도록 해준다
 * */
OpenManipulatorTeleop::OpenManipulatorTeleop()
    : node_handle_(""), priv_node_handle_("~") {
  /************************************************************
  ** Initialize variables
  ************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  initClient();

  disableWaitingForEnter();

  // ROS_INFO:  macro is used for logging informational messages.
  ROS_INFO("OpenManipulator teleoperation using keyboard start");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop() {
  restoreTerminalSettings();
  ROS_INFO("Terminate OpenManipulator Joystick");
  ros::shutdown();
}

// To get services
void OpenManipulatorTeleop::initClient() {
  // Getting the goal joint space
  goal_joint_space_path_client_ =
      node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>(
          "goal_joint_space_path");

  // Getting the goal joint space from present client
  goal_joint_space_path_from_present_client_ =
      node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>(
          "goal_joint_space_path_from_present");

  // Getting the goal task space from present position only client
  goal_task_space_path_from_present_position_only_client_ =
      node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>(
          "goal_task_space_path_from_present_position_only");

  // Getting the goal tool control client
  goal_tool_control_client_ =
      node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>(
          "goal_tool_control");
}

void OpenManipulatorTeleop::initSubscriber() {
  /*
   * ROS1 node_handle_.subscribe
   * 1. 초기화된 node_handle_ 을 통해, `joint_states` Topic을 구독
   * 이후 `joint_state` Topic을 통해 들어오는 메시지를 받을 수 있게 된다
   * 2. queue를 이용해 최대 10개의 메시지를 버퍼링 한다
   * 3. 메시지가 수신되면, 실행될 callback 함수를 포인터 형식으로 받는다
   * 4. 객체의 멤버함수를 callback 함수로 지정하고자 할때, 그 객체의 주소를
   * 넘겨주어야 한다.
   */
  joint_states_sub_ = node_handle_.subscribe(
      "joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe(
      "kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback,
      this);
}

// Topic이 호출 -> Subscriber Node 에게 메시지 전파 -> Callback 실행
// JointState, 관절 상태에 대한 정보를 받는다
void OpenManipulatorTeleop::jointStatesCallback(
    const sensor_msgs::JointState::ConstPtr &msg) {
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  // Message의 새로운 관절 위치를 사용해,
  // present_joint_angle 업데이트
  // name: ["joint1", "joint2", "joint3", "joint4"]
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i++) {
    if (!msg->name.at(i).compare("joint1"))
      // i번째 관절 이름이 "joint1" 이면,
      // -> Message의 i번째 위치 불러옴
      // -> 첫번째 관절 각도 조절
      temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))
      temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))
      temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))
      temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorTeleop::kinematicsPoseCallback(
    const open_manipulator_msgs::KinematicsPose::ConstPtr &msg) {
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle() {
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose() {
  return present_kinematic_position_;
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(
    std::vector<std::string> joint_name, std::vector<double> joint_angle,
    double path_time) {
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv)) {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePath(
    std::vector<std::string> joint_name, std::vector<double> joint_angle,
    double path_time) {
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv)) {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle) {
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(
      priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv)) {
    return srv.response.is_planned;
  }
  return false;
}

/*
 *
 * TaskSpacePath: 작업공간의 경로
 * PresentPosition: 현재 위치
 *
 * 현재 위치를 기준으로, 작업공간의 경로 설정
 * `kinematics_pose` : X,Y,Z position
 * `path_time` : the time to complete the movement the the new position
 * */
bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(
    std::vector<double> kinematics_pose, double path_time) {

  // Controlling the OpenManipulator robot arm by Kinematics Position
  open_manipulator_msgs::SetKinematicsPose srv;

  /* Setting to the name of the end effector.
   *
   * planning_group :
   *
   * - retrieve `"end_effector_name"` from the ROS parameter server.
   * - if it's not set, it defaults to `"gripper"`
   */
  srv.request.planning_group =
      priv_node_handle_.param<std::string>("end_effector_name", "gripper");

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if (goal_task_space_path_from_present_position_only_client_.call(srv)) {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::printText() {
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your OpenManipulator!\n");
  printf("---------------------------\n");
  printf("w : increase x axis in task space\n");
  printf("s : decrease x axis in task space\n");
  printf("a : increase y axis in task space\n");
  printf("d : decrease y axis in task space\n");
  printf("z : increase z axis in task space\n");
  printf("x : decrease z axis in task space\n");
  printf("\n");
  printf("y : increase joint 1 angle\n");
  printf("h : decrease joint 1 angle\n");
  printf("u : increase joint 2 angle\n");
  printf("j : decrease joint 2 angle\n");
  printf("i : increase joint 3 angle\n");
  printf("k : decrease joint 3 angle\n");
  printf("o : increase joint 4 angle\n");
  printf("l : decrease joint 4 angle\n");
  printf("\n");
  printf("g : gripper open\n");
  printf("f : gripper close\n");
  printf("       \n");
  printf("1 : init pose\n");
  printf("2 : home pose\n");
  printf("3 : src pose\n");
  printf("4 : src pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0), getPresentJointAngle().at(1),
         getPresentJointAngle().at(2), getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0), getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");
}

// Set the destination goal position
void OpenManipulatorTeleop::setGoal(char ch) {
  std::vector<double> goalPose;
  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint;
  goalJoint.resize(NUM_OF_JOINT, 0.0);

  // Control Potision of Robot arm
  //============================================================
  //===[ Control by position ]===
  //============================================================
  if (ch == 'w' || ch == 'W') {
    printf("input : w \tincrease(++) x axis in task space\n");
    goalPose.at(0) = DELTA; // Position Delta
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  } else if (ch == 's' || ch == 'S') {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  } else if (ch == 'a' || ch == 'A') {
    printf("input : a \tincrease(++) y axis in task space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  } else if (ch == 'd' || ch == 'D') {
    printf("input : d \tdecrease(--) y axis in task space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  } else if (ch == 'z' || ch == 'Z') {
    printf("input : z \tincrease(++) z axis in task space\n");
    goalPose.at(2) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  } else if (ch == 'x' || ch == 'X') {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goalPose.at(2) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
    //============================================================
    //===[ Control by joint angle ]===
    //============================================================
  } else if (ch == 'y' || ch == 'Y') {
    printf("input : y \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    goalJoint.at(0) = JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  } else if (ch == 'h' || ch == 'H') {
    printf("input : h \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    goalJoint.at(0) = -JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  } else if (ch == 'u' || ch == 'U') {
    printf("input : u \tincrease(++) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    goalJoint.at(1) = JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  } else if (ch == 'j' || ch == 'J') {
    printf("input : j \tdecrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    goalJoint.at(1) = -JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  } else if (ch == 'i' || ch == 'I') {
    printf("input : i \tincrease(++) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    goalJoint.at(2) = JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  } else if (ch == 'k' || ch == 'K') {
    printf("input : k \tdecrease(--) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    goalJoint.at(2) = -JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  } else if (ch == 'o' || ch == 'O') {
    printf("input : o \tincrease(++) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    goalJoint.at(3) = JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  } else if (ch == 'l' || ch == 'L') {
    printf("input : l \tdecrease(--) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    goalJoint.at(3) = -JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  } else if (ch == 'g' || ch == 'G') {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  } else if (ch == 'f' || ch == 'F') {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
    //============================================================
    //===[ Specified Position ]===
    //============================================================
  } else if (ch == '2') {
    printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1");
    joint_angle.push_back(0.0);
    joint_name.push_back("joint2");
    joint_angle.push_back(-1.05);
    joint_name.push_back("joint3");
    joint_angle.push_back(0.35);
    joint_name.push_back("joint4");
    joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
  } else if (ch == '1') {
    printf("input : 1 \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1");
    joint_angle.push_back(0.0);
    joint_name.push_back("joint2");
    joint_angle.push_back(0.0);
    joint_name.push_back("joint3");
    joint_angle.push_back(0.0);
    joint_name.push_back("joint4");
    joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  } else if (ch = '3') {
    printf("input : 3 \tsrc pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1");
    joint_angle.push_back(-0.502);
    joint_name.push_back("joint2");
    joint_angle.push_back(0.462);
    joint_name.push_back("joint3");
    joint_angle.push_back(-0.324);
    joint_name.push_back("joint4");
    joint_angle.push_back(0.902);

    setJointSpacePath(joint_name, joint_angle, path_time);

    // printf("1. close gripper\n");
    // printf("2. move to upper side\n");
    // printf("3. move to dest\n");
    // printf("4. move to lower side\n");
    // printf("5. open gripper\n");
  } else if (ch = '4') {
    printf("input : 4 \tsecond pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1");
    joint_angle.push_back(-0.500);
    joint_name.push_back("joint2");
    joint_angle.push_back(0.451);
    joint_name.push_back("joint3");
    joint_angle.push_back(-0.314);
    joint_name.push_back("joint4");
    joint_angle.push_back(0.904);

    setJointSpacePath(joint_name, joint_angle, path_time);
  } else if (ch = '5') {
    printf("input : 5 \tthird pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1");
    joint_angle.push_back(1.612);
    joint_name.push_back("joint2");
    joint_angle.push_back(-0.233);
    joint_name.push_back("joint3");
    joint_angle.push_back(0.412);
    joint_name.push_back("joint4");
    joint_angle.push_back(1.203);

    setJointSpacePath(joint_name, joint_angle, path_time);
  } else if (ch = '6') {
    printf("input : 6 \tfourth pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1");
    joint_angle.push_back(1.615);
    joint_name.push_back("joint2");
    joint_angle.push_back(-0.167);
    joint_name.push_back("joint3");
    joint_angle.push_back(0.480);
    joint_name.push_back("joint4");
    joint_angle.push_back(1.256);

    setJointSpacePath(joint_name, joint_angle, path_time);
  }
}

void OpenManipulatorTeleop::restoreTerminalSettings(void) {
  tcsetattr(0, TCSANOW, &oldt_); /* Apply saved settings */
}

void OpenManipulatorTeleop::disableWaitingForEnter(void) {
  // `termios` is used to store the terminal I/O setting
  struct termios newt;

  /* Getting settings
   *
   * 0: Getting the current terminal I/O settings for file descipter `0`
   * &oldt_: Settings will be saved into a class member variable `&oldt_`
   */
  tcgetattr(0, &oldt_);

  // Init new settings
  newt = oldt_;

  /* Set new configs
   *
   * ICANON:
   * - input is started immediately without waiting for a newline (Canonical)
   *
   * ECHO: Echoing input characters to the terminal
   */
  newt.c_lflag &= ~(ICANON | ECHO);

  /* Apply settings
   *
   * 0: Applying for the file descripter `0`
   * TCSANOW: Specifying that the change should take effect immediately.
   * &newt: Terminal settings
   * */
  tcsetattr(0, TCSANOW, &newt);
}

int main(int argc, char **argv) {
  /* Init ROS node
   *
   * argc, argv:
   * - argument parsing such as `__name`, `__log`, `__ns`, etc.
   * - These can override the node name, loggign directory, and namespace.
   *
   * `__name` : name of the node
   * `__master` : ROS master info (ROS_MASTER_URI)
   *
   * "open_manipulator_teleop_keyboard":
   * - defalt name of the node, `__name` in argv
   * */
  ros::init(argc, argv, "open_manipulator_teleop_keyboard");

  // create teleoperation handler object
  OpenManipulatorTeleop openManipulatorTeleop;

  char ch;
  // print Help text for user
  openManipulatorTeleop.printText();

  /* Getting keyboard input and operate it.
   *
   * `ros::ok()` : a function that returns `true` if ROS is running properly
   * `q` : quit
   * */
  while (ros::ok() && (ch = std::getchar()) != 'q') {
    ros::spinOnce(); // Processes callbackes once (what callbacks?)
    openManipulatorTeleop.printText();
    ros::spinOnce();
    openManipulatorTeleop.setGoal(ch); //
  }

  return 0;
}
