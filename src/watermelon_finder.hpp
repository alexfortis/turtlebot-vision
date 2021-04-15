#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <tuple>

#define NAV_DEBUG 1

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef std::tuple<double, double, double> Goal;

bool found_watermelon();

bool moveToGoal(MoveBaseClient &ac, const double &x, const double &y, const double angle);
