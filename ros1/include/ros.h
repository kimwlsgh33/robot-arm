#include <iostream>

// sensor_msgs::JointState joint_state;
// joint_state.header.stamp = ros::Time::now();
// joint_state.name = {"joint1", "joint2", "joint3", "joint4"};
// joint_state.position = {1.0, 0.5, -0.5, 0.0};
// joint_state.velocity = {0.1, 0.1, 0.1, 0.1};
// joint_state.effort = {0.0, 0.0, 0.0, 0.0};
namespace sensor_msgs {
class JointState {
public:
  struct ConstPtr {
  public:
    std::string name;
  };
};
} // namespace sensor_msgs

int test(void) {
  const sensor_msgs::JointState::ConstPtr &msg =
      malloc(sensor_msgs::JointState::ConstPtr);
  std::cout << src->name;
}
