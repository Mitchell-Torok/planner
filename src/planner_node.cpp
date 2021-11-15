#include "rclcpp/rclcpp.hpp"
#include "comp3431_interfaces/srv/map_info.hpp"
#include "comp3431_interfaces/action/move_object_to_room.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "plan_parser.hpp"
//include "planner/planner_node.hpp"

#include <memory>
#include <string>
#include <functional>
#include <thread>

  
class PlannerNode : public rclcpp::Node {


public:
	PlannerNode() : Node("PlannerNode") { 
	
	using namespace std::placeholders;

	    this->action_server_ = rclcpp_action::create_server<comp3431_interfaces::action::MoveObjectToRoom>(
	      this, "move_object_to_room",
	      std::bind(&PlannerNode::handle_goal, this, _1, _2),
	      std::bind(&PlannerNode::handle_cancel, this, _1),
	      std::bind(&PlannerNode::handle_accepted, this, _1));
	      
      
		service_ = this->create_service<comp3431_interfaces::srv::MapInfo>( "set_map_info", std::bind(&PlannerNode::set_map_info, this, _1, _2));
	}


private:
	void set_map_info(
		const std::shared_ptr<comp3431_interfaces::srv::MapInfo::Request> request,
		const std::shared_ptr<comp3431_interfaces::srv::MapInfo::Response> response)
	{
	  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %s", request->blocks[0].text.c_str());
	  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %s", request->blocks[1].text.c_str());
	}
	

	rclcpp_action::GoalResponse handle_goal(
	  const rclcpp_action::GoalUUID & uuid,
	  std::shared_ptr<const comp3431_interfaces::action::MoveObjectToRoom::Goal> goal)
	{
	  RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->room.c_str());
	  RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->object.c_str());
	  system("gnome-terminal -- sh -c './src/planner/FF-X/ff -o src/planner/FF-X/tyreworld_domain.pddl -f src/planner/FF-X/tyreworld_facts1 > src/planner/src/plan.txt'");
	  
	  
	  //PlanParser plan = new PlanParse("plan.txt");
	  //RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", PlanParser.getNextStep());
	  
	  (void)uuid;
	  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}



	rclcpp_action::CancelResponse handle_cancel(
	  const std::shared_ptr< rclcpp_action::ServerGoalHandle<comp3431_interfaces::action::MoveObjectToRoom>> goal_handle)
	{
	  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
	  (void)goal_handle;
	  return rclcpp_action::CancelResponse::ACCEPT;
	}


	void handle_accepted(const std::shared_ptr< rclcpp_action::ServerGoalHandle<comp3431_interfaces::action::MoveObjectToRoom>> goal_handle)
	{
	  using namespace std::placeholders;
	  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
	  std::thread{std::bind(&PlannerNode::execute, this, _1), goal_handle}.detach();
	}

	
	rclcpp::Service<comp3431_interfaces::srv::MapInfo>::SharedPtr service_;
	rclcpp_action::Server<comp3431_interfaces::action::MoveObjectToRoom>::SharedPtr action_server_;
	
	void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comp3431_interfaces::action::MoveObjectToRoom>> goal_handle) {
    		RCLCPP_INFO(this->get_logger(), "Executing goal");

	}
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
}
