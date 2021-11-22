#include "rclcpp/rclcpp.hpp"
#include "comp3431_interfaces/srv/map_info.hpp"
#include "comp3431_interfaces/action/move_object_to_room.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "comp3431_interfaces/msg/qr_code_block.hpp"
#include "planner/plan_parser.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

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
	      
	    this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,"navigate_to_pose");    		
	    service_ = this->create_service<comp3431_interfaces::srv::MapInfo>( "set_map_info", std::bind(&PlannerNode::set_map_info, this, _1, _2));
	}


private:
	//std::string block[10] = {""};
	//int numRooms = 0;
	bool fin = false;
	bool abort = false;
	std::vector<comp3431_interfaces::msg::QRCodeBlock> blocks;
	 
	//Sets map info when a new command comes in 
	void set_map_info(
		const std::shared_ptr<comp3431_interfaces::srv::MapInfo::Request> request,
		const std::shared_ptr<comp3431_interfaces::srv::MapInfo::Response> response)
	
	{	
	  //numRooms = request->blocks.size();  
	  for (int i = 0; i < int(request->blocks.size()); i ++) {
	  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incomming Text: %s", request->blocks[i].text.c_str());
	  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incomming x:    %f", request->blocks[i].pose.position.x);
	  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incomming y:    %f", request->blocks[i].pose.position.y);
	  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incomming z:    %f", request->blocks[i].pose.position.z);
	  	
	  	auto block = comp3431_interfaces::msg::QRCodeBlock();
		block.text = request->blocks[i].text.c_str();
		block.pose.position.x = request->blocks[i].pose.position.x;
		block.pose.position.y = request->blocks[i].pose.position.y;
		block.pose.position.z = request->blocks[i].pose.position.z;
		  
		blocks.push_back(block);
	  }
	}
	

	rclcpp_action::GoalResponse handle_goal(
	  const rclcpp_action::GoalUUID & uuid,
	  std::shared_ptr<const comp3431_interfaces::action::MoveObjectToRoom::Goal> goal)
	{
	  RCLCPP_INFO(this->get_logger(), "Received goal request with room %s", goal->room.c_str());
	  RCLCPP_INFO(this->get_logger(), "Received goal request with object %s", goal->object.c_str());
	  
	  
	//Problem Generation 
	std::string problemPath = "problem.pddl";
	std::ofstream file;
	file.open(problemPath);
	file << "(define (problem moveitemtoroom)" << std::endl;
	file << "  (:domain turtlebot3-domain)" << std::endl;
	
	
	//OBJECTS
	file << "  (:objects" << std::endl;
	file << "	turtlebot - robot" << std::endl;
	file << "	initial-room - room" << std::endl;
	//Add Extra Objects e.g. Rooms and Items
	
	int i = 0;
	int m = 0;
	
	while (i < int(blocks.size())) {	
		m = 0;
		
		std::string str = blocks[i].text;
		char *cstr = new char[str.length() + 1];
		strcpy(cstr, str.c_str());
		char *word;
		word = strtok(cstr, " ");
		while(word != NULL) {
			if (m == 0) {
				file << "	";
				file << word;
				file << " - room" << std::endl;
			} else {	
				file << "	";
				file << word;
				file << " - item" << std::endl;
			}
			m++;
			word = strtok(NULL, " ");	
		}
		i++;		
	}
	file << "  )" << std::endl;
	file << "\n";
	
	//INIT
	file << "  (:init" << std::endl;
	//Add Initial Items to their rooms
	//ONLY EXAMPLE BELOW
	i = 0;
	m = 0;
	file << "	(at turtlebot initial-room)" << std::endl;
	while (i < int(blocks.size())) {
		m = 0;
		std::string str = blocks[i].text;
		char *cstr = new char[str.length() + 1];
		strcpy(cstr, str.c_str());
		char *word = strtok(cstr, " ");
		char *room;
		while (word != NULL) {
			if (m == 0) {
				file << "	(in ";
				room = word;
			} else {
				file << word;
				file << " ";
			}
			word = strtok(NULL, " ");	
			m++;		
		}
		file << room;
		file << ")" << std::endl;
		i++;
	}
	file << "	(hand_empty turtlebot)" << std::endl;
	file << "  )" << std::endl;
	file << "\n";
	//GOAL
	//file << " ";
	file << "  (:goal" << std::endl;
	//Add Goals from MoveObjectToRoom
	file << "	(in ";
	file << goal->object.c_str();
	file << " ";
	file << goal->room.c_str();
	file << ")" << std::endl;
	//goal->room.c_str()
	//goal->object.c_str()
	
	file << "  )" << std::endl;
	
	
	//End of file
	file << ")" << std::endl;
	file.close();
	
	system("gnome-terminal -- sh -c './src/planner/FF-X/ff -o src/planner/FF-X/domain.pddl -f problem.pddl > src/planner/src/solution.txt'");	  
		  
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
	rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
	void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comp3431_interfaces::action::MoveObjectToRoom>> goal_handle) {
    		RCLCPP_INFO(this->get_logger(), "Executing goal");
    		const auto goal = goal_handle->get_goal();
    		auto feedback = std::make_shared<comp3431_interfaces::action::MoveObjectToRoom::Feedback>();
    		PlanParser *plan = new PlanParser("/home/rsa2021/Ass3_ws/src/planner/src/solution.txt"); 
    		PlanParser *plan1 = new PlanParser("/home/rsa2021/Ass3_ws/src/planner/src/solution.txt"); 
    		std::cout << "-----------Generated Plan-----------" << std::endl;
    		bool firstMove = false;
    		
    		while (!(plan1->done())) {
    			std::vector<std::string> next = plan1->getNextStep();
    			
    			std::cout << next[0].c_str() << " " <<next[1].c_str() << " " << next[2].c_str() << " " <<next[3].c_str() << std::endl;
    			
		   
    		}
    		std::cout << "---------End Generated Plan--------- " << std::endl;
    		
    		
    		while(!(plan->done())) {
    			//RCLCPP_INFO(this->get_logger(), "planning");
    			
    			std::vector<std::string> next = plan->getNextStep();
    			//RCLCPP_INFO(this->get_logger(), "%s", next[0].c_str());
    			
    			//RCLCPP_INFO(this->get_logger(), "%s", plan->getNextStep[1]);

				//plan = planParser.getNextStep;
    			if (goal_handle->is_canceling()) {
				//result = sequence;
				//goal_handle->canceled();
				RCLCPP_INFO(this->get_logger(), "Goal canceled");
				return;
		      }
		      //next[0].c_str().compare("pick");
		      std::string str1 = next[0].c_str();
		      std::string str2 = "pick";
		      if (str1.compare(str2) == 0) {
		      	std::cout << "pick turtlebot " << next[2].c_str() << " " <<next[3].c_str() << std::endl;
		      	//RCLCPP_INFO(this->get_logger(), "pick turtlebot %s %s", next[2].c_str(), next[3].c_str());
		      	
		      }
		      str2 = "place";
		      if (str1.compare(str2) == 0) {
		      	std::cout << "place turtlebot " << next[2].c_str() << " " <<next[3].c_str() << std::endl;
		      	//RCLCPP_INFO(this->get_logger(), "place turtlebot %s %s", next[2].c_str(), next[3].c_str());

		      	
		      }
		      //WHen requested to move send that action
		      str2 = "move";
		      
		      if (str1.compare(str2) == 0) {
		      	std::cout << "move turtlebot " << next[2].c_str() << " " <<next[3].c_str() << std::endl;	
		      	auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    			int i = 0;
    			while (i < int(blocks.size())) {  
					std::string str = blocks[i].text;
					char *cstr = new char[str.length() + 1];
					strcpy(cstr, str.c_str());
					std::string room = strtok(cstr, " ");
    				std::string location = next[3].c_str();
    				if (room.compare(location.c_str()) == 0) {
    					goal_msg.pose.pose.position.x = blocks[i].pose.position.x;
    					goal_msg.pose.pose.position.y = blocks[i].pose.position.y;
    					goal_msg.pose.pose.position.z = blocks[i].pose.position.z;
    					RCLCPP_INFO(this->get_logger(), "x = %f, y = %f, z = %f", goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, goal_msg.pose.pose.position.z);
    					break;
    				}
    				i++;
    			}
    		
    		/*
    		if (firstMove == false) {
    			goal_msg.pose.pose.position.x = 0.0;
    			goal_msg.pose.pose.position.y = -1.5;
    			goal_msg.pose.pose.position.z = 0.0;
    			firstMove = true;
    		} else {
    			goal_msg.pose.pose.position.x = -2.0;
    			goal_msg.pose.pose.position.y = 1.0;
    			goal_msg.pose.pose.position.z = 0.0;
    		}
    		*/
    			auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    			send_goal_options.goal_response_callback = std::bind(&PlannerNode::goal_response_callback, this, std::placeholders::_1);
    			send_goal_options.feedback_callback = std::bind(&PlannerNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    			send_goal_options.result_callback = std::bind(&PlannerNode::result_callback, this, std::placeholders::_1);
    		
    			this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    			while(fin == false) {
    				
    				if (abort == true) {
    					this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    					abort = false;
    				}
    				//RCLCPP_INFO(this->get_logger(), "%d", fin);
    			}
    			fin = false;
		      goal_handle->publish_feedback(feedback);
      		      RCLCPP_INFO(this->get_logger(), "Publish feedback");
    		
    		}
		      
		      }
		      /*
		      if (!this->client_ptr_->wait_for_action_server()) {
      			RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      			rclcpp::shutdown();
    		  	}
    		  */
    		
    		std::cout << "---------End Generated Plan--------- " << std::endl;
    		//RCLCPP_INFO(this->get_logger(), "doneee");

	}
	
	  void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      //RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      //RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    std::stringstream ss;
    
    ss << "Next number in sequence received: ";
    //RCLCPP_INFO(this->get_logger(), "dist = %f" ,feedback->distance_remaining);
//    for (auto number : feedback->partial_sequence) {
 //     ss << number << " ";
//    }
    
    //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    
    //RCLCPP_INFO(this->get_logger(), "feedbackcall");
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
  	//RCLCPP_INFO(this->get_logger(), "result = %s, result.result = %s", result.code, result.result);
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_ERROR(this->get_logger(), "Goal was successful");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        abort = true;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    fin = true;
    ss << "Result received: ";
//    for (auto number : result.result->sequence) {
//      ss << number << " ";
//    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    
    //RCLCPP_INFO(this->get_logger(), "Shutdown");
    //rclcpp::shutdown();
  }
  
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
}
