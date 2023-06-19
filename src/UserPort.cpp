/* @brief UserPort means the Config port */

/* Include ROS headers */
#include "ros/ros.h"

/* Include standard C/C++ headers */
#include <cstdio>
#include <cstdlib>
#include <fstream>    // for file stream
#include <regex>

/* Include awr1843aop package custom headers */
#include "awr1843aop/sensorCLI.h"
#include "awr1843aop/Parser.hpp"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "UserPort");
	ros::NodeHandle nh;
	awr1843aop::sensorCLI srv;

	if (argc != 2) {
		ROS_INFO("Your command is improper, so check your arguments");
		return 1;
	}
	else
		ROS_INFO("Using config file: %s", argv[1]);

	ros::ServiceClient client = nh.serviceClient<awr1843aop::sensorCLI>("/sensorCLI");
	std::ifstream cfgFile;
	awr1843aop::ParameterParser parser;

	// wait for service to become available
	ros::service::waitForService("/sensorCLI", 10000);

	// wait 0.5 secs to avoid multi-sensor conflicts
	ros::Duration(0.5).sleep();
	
	cfgFile.open(argv[1]);

	if (cfgFile.is_open()) {
		while (std::getline(cfgFile, srv.request.req)) {
			// Remove Windows carriage-return if present
			srv.request.req.erase(std::remove(srv.request.req.begin(), srv.request.req.end(), '\r'), srv.request.req.end());
			// Ignore comment lines (first non-space char is '%') or blank lines
			if (!(std::regex_match(srv.request.req, std::regex("^\\s*%.*")) || std::regex_match(srv.request.req, std::regex("^\\s*")))) {
				
				// Request: Send srv object to Server 
				if (client.call(srv)) {
					if (std::regex_search(srv.response.resp, std::regex("Done"))) {
						// Server should be including "Done" in the resp object
						parser.ParamsParser(srv, nh);
					}
					else {
						ROS_ERROR("Response wrong (awr1843aop sensor did not respond with 'Done')");
						ROS_ERROR("Rsponse: '%s'", srv.response.resp.c_str());
						return 1;
					}


				}
				else {
					ROS_ERROR("Service request failed");
					ROS_ERROR("%s", srv.request.req.c_str());
					return 1;
				}
			}
		}
		parser.CalParams(nh);
		cfgFile.close();
	}
	else {
		ROS_ERROR("Failed to open config file (%s)", argv[1]);
		return 1;
	}

	ROS_INFO("Done configuring awr1843aop device using config file: %s", argv[1]);
	ROS_INFO("UserPort process will terminate.");
	return 0;
}
	
