#ifndef PLANNER__PLANNER_NODE_HPP_
#define PLANNER__PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "comp3431_interfaces/srv/map_info.hpp"

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  void set_map_info(comp3431_interfaces::srv::MapInfo::Request::ConstSharedPtr msg);

  rclcpp::Service<comp3431_interfaces::srv::MapInfo>::SharedPtr service_;
   
};

#endif
