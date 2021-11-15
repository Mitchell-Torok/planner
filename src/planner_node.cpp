#include "rclcpp/rclcpp.hpp"
#include "comp3431_interfaces/srv/map_info.hpp"
#include "comp3431_interfaces/action/move_object_to_room.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "plan_parser.hpp"
//include "planner/planner_node.hpp"

#include <fstream>
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
	std::string block[10] = {""};
	int numRooms = 0;
	 
	void set_map_info(
		const std::shared_ptr<comp3431_interfaces::srv::MapInfo::Request> request,
		const std::shared_ptr<comp3431_interfaces::srv::MapInfo::Response> response)
	
	{	
	  int i = 0;	  
	  while (i < request->blocks.size()) {
	  	//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %s", request->blocks[i].text.c_str());
	  	
	  	block[i] = request->blocks[i].text;
	  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %s", block[i].c_str());
	  	i++;
	  }
	  
	  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %d", request->blocks.size());
	  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %s", request->blocks[1].text.c_str());
	}
	

	rclcpp_action::GoalResponse handle_goal(
	  const rclcpp_action::GoalUUID & uuid,
	  std::shared_ptr<const comp3431_interfaces::action::MoveObjectToRoom::Goal> goal)
	{
	  RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->room.c_str());
	  RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->object.c_str());
	  system("gnome-terminal -- sh -c './src/planner/FF-X/ff -o src/planner/FF-X/tyreworld_domain.pddl -f src/planner/FF-X/tyreworld_facts1 > src/planner/src/plan.txt'");
	  
	  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request %s", block[0].c_str());
	  //PlanParser plan = new PlanParse("plan.txt");
	  //RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", PlanParser.getNextStep());
		  
	//Problem Generation 
	std::string problemPath = "problem.pddl";
	std::ofstream file;
	file.open(problemPath);
	file << "(define (problem moveitemtoroom)" << std::endl;
	file << " (:domain turtlebot3-domain)" << std::endl;
	
	/*
	//OBJECTS
	file << " (:objects" << std::endl;
	file << " turtlebot - robot" << std::endl;
	//Add Extra Objects e.g. Rooms and Items
	
	int i = 0;
	int m = 0;
	
	while (request->blocks[i].text != NULL) {
		m = 0;
		char *string = request->blocks[i].text;
		char *word;
		word = strtok(string, " ");
		
		while(word != NULL) {
			if (m == 0) {
				file << word;
				file << " - room" << std::endl;
			} else {	
				file << word;
				file << " - item" << std::endl;
			}
			m++;
			word = strtok(NULL, " ");	
		}
		i++;
	}
	
	file << ")" << std::endl;
	
	
	//INIT
	file << "(:init" << std::endl;
	//Add Initial Items to their rooms
	//ONLY EXAMPLE BELOW
	i = 0;
	m = 0;
	//How to set initial room?
	while (request->blocks[i].text != NULL) {
		m = 0;
		char *string = request->blocks[i].text;
		char *word = strtok(string, " ");;
		char *room;
		while (word != NULL) {
			if (m == 0) {
				file << "(in ";
				room = word;
			
			} else {
				file << "%s ", word;
			
			}
			word = strtok(NULL, " ");	
			m++;		
		}
		file << "%s)", room << std::endl;
		i++;
	}
	file << "(hand_empty turtlebot)" << std::endl;
	//file << " (at turtlebot initial-room)" << std::endl;
	file << ")" << std::endl;
	
	
	
	//GOAL
	//file << " ";
	file << "(:goal" << std::endl;
	//Add Goals from MoveObjectToRoom
	file << "(in " << std::endl;
	file << "%s ", goal->object.c_str();
	file << "%s)", goal->room.c_str() << std::endl;
	//goal->room.c_str()
	//goal->object.c_str()
	
	file << ")" << std::endl;
	
	*/
	//End of file
	file << ")" << std::endl;
	file.close();
		  
		  
	  //Problem Generation ^^
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
