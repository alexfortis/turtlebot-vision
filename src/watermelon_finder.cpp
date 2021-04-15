#include "watermelon_finder.hpp"

using std::vector;
using std::make_tuple;

bool found_watermelon() {
  return false;
}

std::ostream &operator<<(std::ostream &os, const Goal &g) {
  os << "(" << std::get<0>(g) << ", " << std::get<1>(g) << ", " << std::get<2>(g) << ")";
  return os;
}

bool moveToGoal(MoveBaseClient &ac, const double &x, const double &y, const double angle=0.0) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = angle;

#if NAV_DEBUG
  std::cout << "Sending goal: (" << x << ", " << y << "); " << angle << std::endl;
#endif
  ROS_INFO("Sending goal: (%.1f %.1f) at angle %.1f\n", x, y, angle);
  ac.sendGoal(goal);
#if NAV_DEBUG
  std::cout << "Sent goal." << std::endl;
  bool rslt =
#endif
  ac.waitForResult();
#if NAV_DEBUG
  std::cout << "Result: " << rslt << std::endl;
#endif
  return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

int main(int argc, char **argv) {
  std::cout << "Watermelon finder not yet implemented" << std::endl;
  
  ros::init(argc, argv, "watermelon_finder");
  MoveBaseClient action_client("move_base", true);

  double x0 = 5.42, y0 = 2.72,
    x1 = 4.05, y1 = -4.43,
    x2 = -6.38, y2 = -1.5;
  vector<Goal> goals;
  goals.push_back(make_tuple(5.42, 2.72, 5.0));
  goals.push_back(make_tuple(4.05, -4.43, 1.0));
  goals.push_back(make_tuple(-6.38, -1.5, 1.0));

  bool moved = false;
  double goal_x, goal_y, goal_angle;
  
  for(auto it = goals.begin(); it != goals.end(); ++it) {
    std::tie(goal_x, goal_y, goal_angle) = *it;
#if NAV_DEBUG
    std::cout << "Moving to (" << goal_x << ", " << goal_y << ")" << std::endl;
    
#endif
    moved = moveToGoal(action_client, goal_x, goal_y, goal_angle);
    if(moved) {
      ROS_INFO_STREAM("Successfully moved to " << *it);
      ROS_INFO("%s", found_watermelon()?"True":"False");
    }
    else {
#if NAV_DEBUG
      std::cout << "Failed to move to " << *it << std::endl;
#endif
      ROS_INFO_STREAM("Failed to move to " << *it);
    }
  }
}
