#include "rclcpp/rclcpp.hpp"
#include "comp3431_interfaces/srv/map_info.hpp"
//include "planner/planner_node.hpp"

#include <memory>
#include <string>
#include <functional>

  
class PlannerNode : public rclcpp::Node {


public:
	PlannerNode() : Node("PlannerNode") { 
		service_ = this->create_service<comp3431_interfaces::srv::MapInfo>(
		"set_map_info", std::bind(&PlannerNode::set_map_info, this, std::placeholders::_1, std::placeholders::_2));
	}


private:

	void set_map_info(
		const std::shared_ptr<comp3431_interfaces::srv::MapInfo::Request> request,
		const std::shared_ptr<comp3431_interfaces::srv::MapInfo::Response> response)
	{

	  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %s", request->blocks[0].text.c_str());
	  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %s", request->blocks[1].text.c_str());


	}
	
	rclcpp::Service<comp3431_interfaces::srv::MapInfo>::SharedPtr service_;
	
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
}
