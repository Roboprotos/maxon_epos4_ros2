/**
 * Copyright 2022 by maxon motor ag
 * All rights reserved.
 * @file cpp_example_1dof_ppm.cpp
 * @author Cyril Jourdan
 * @date Mar 21, 2022
 * @brief Example code to control EPOS4 with ROS2 through ros1_bridge and ros_canopen on ROS1 side
 * Contact: cyril.jourdan@roboprotos.com
 */

#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std;

/*** global variables ***/
sensor_msgs::msg::JointState joint_state;
bool js_arrived = false;


/** @brief Callback function for the Joint State subscriber */
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  joint_state = *msg;
  js_arrived = true;
}

/** @brief main function */
int main(int argc, char ** argv)
{
	// Initialize the ROS node
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("maxon_epos4_ros1_bridge_cpp_example");

        // Variables
        int menu_i = 0; // integer to store the menu choice
        int cmd_pos_i = 0; // integer to store the target position
        std_msgs::msg::Float64 cmd_pos; // float standard message corresponding to the command topic
        cmd_pos.data = 0;
	auto request = std::make_shared<std_srvs::srv::Trigger::Request>(); // Trigger request variable for calling driver services
        bool exit = false; // integer for controlling the loop

        // Service clients
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_driver_init = node->create_client<std_srvs::srv::Trigger>("/maxon/driver/init");
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_driver_halt = node->create_client<std_srvs::srv::Trigger>("/maxon/driver/halt");
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_driver_recover = node->create_client<std_srvs::srv::Trigger>("/maxon/driver/recover");
	rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clt_driver_shutdown = node->create_client<std_srvs::srv::Trigger>("/maxon/driver/shutdown");

        // Topic Publisher
        auto pub_pos = node->create_publisher<std_msgs::msg::Float64>("/maxon/canopen_motor/base_link1_joint_position_controller/command", 1);

        // Topic Subscriber
        auto sub_js = node->create_subscription<sensor_msgs::msg::JointState>("/maxon/joint_states", 1, joint_state_callback);

	// Display UI in terminal
        RCLCPP_INFO(node->get_logger(), "********************************************");
        RCLCPP_INFO(node->get_logger(), "maxon EPOS4 - ROS 1 ros_canopen example code");
        RCLCPP_INFO(node->get_logger(), "To be used with 1 EPOS4 with Node-ID 1");
        RCLCPP_INFO(node->get_logger(), "To run after roslaunch of a PPM example");
        RCLCPP_INFO(node->get_logger(), "********************************************");

	RCLCPP_INFO(node->get_logger(), "MENU");
        RCLCPP_INFO(node->get_logger(), "1 - driver init");
        RCLCPP_INFO(node->get_logger(), "2 - driver halt");
        RCLCPP_INFO(node->get_logger(), "3 - driver recover");
        RCLCPP_INFO(node->get_logger(), "4 - driver shutdown");
        RCLCPP_INFO(node->get_logger(), "5 - set Target Position");
        RCLCPP_INFO(node->get_logger(), "6 - get Position Actual Value");
        RCLCPP_INFO(node->get_logger(), "7 - Exit");

	while(!exit)
        {
                // Ask for user input
                RCLCPP_INFO(node->get_logger(), "Please enter a menu choice:");
                cin >> menu_i;

                switch(menu_i)
                {
                        case 1:
			{
				// call init service
				auto result = clt_driver_init->async_send_request(request);
				
				// Wait for the result
				if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
				{
					if(result.get()->success)
					{
						RCLCPP_INFO(node->get_logger(), "EPOS4 Initialization");
					}
				}
			       	else
				{
					RCLCPP_ERROR(node->get_logger(), "Failed to call service");
				}
			}
                        break;

                        case 2:
			{
                                // call halt service
				auto result = clt_driver_halt->async_send_request(request);
				
				// Wait for the result
				if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
				{
					if(result.get()->success)
					{
						RCLCPP_INFO(node->get_logger(), "EPOS4 Halt");
					}
				}
			       	else
				{
					RCLCPP_ERROR(node->get_logger(), "Failed to call service");
				}
			}
			break;

                        case 3:
                 	{       
		 		// call recover service
				auto result = clt_driver_recover->async_send_request(request);
				
				// Wait for the result
				if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
				{
					if(result.get()->success)
					{
						RCLCPP_INFO(node->get_logger(), "EPOS4 Recover");
					}
				}
			       	else
				{
					RCLCPP_ERROR(node->get_logger(), "Failed to call service");
				}
			}
			break;

                        case 4:
			{
				// call shutdown service
				auto result = clt_driver_shutdown->async_send_request(request);
				
				// Wait for the result
				if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
				{
					if(result.get()->success)
					{
						RCLCPP_INFO(node->get_logger(), "EPOS4 Shutdown");
					}
				}
			       	else
				{
					RCLCPP_ERROR(node->get_logger(), "Failed to call service");
				}
			}
			break;

                        case 5:
			{
				RCLCPP_INFO(node->get_logger(), "Enter a Target Position to send to Node-ID 1 in PPM:");
                                cin >> cmd_pos_i;

                                cmd_pos.data = (float)cmd_pos_i; // cast the integer to float and store in command data 

                                // Publish the value to the controller
                                pub_pos->publish(cmd_pos);

                                RCLCPP_INFO(node->get_logger(), "Target Position sent with value = %f", cmd_pos.data);
			}
			break;

                        case 6:
			{
				// Process subscriber callback
				rclcpp::spin_some(node);

                                if(js_arrived) // check if the callback has been executed
                                {
                                        RCLCPP_INFO(node->get_logger(), "Position Actual Value = %f", joint_state.position[0]);

                                        js_arrived = false;
                                }
			}
			break;

                        case 7:
			{
				exit = true; // exit the while loop next time and then the main function
			}
			break;

                        default:
                                RCLCPP_ERROR(node->get_logger(), "Please enter a correct menu value!");
                }
        }

 	rclcpp::shutdown();
	return 0;
}
