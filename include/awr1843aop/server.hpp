#ifndef SENSOR_SERVER
#define SENSOR_SERVER

/* Include ROS headers */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

/* Include standard C/C++ headers */
#include <iostream>
#include <cstdio>
#include <sstream>

/* awr1843aop package custom headers */
#include "awr1843aop/sensorCLI.h"


namespace awr1843aop
{
	class Server : public nodelet::Nodelet
	{
		public:
			Server();

		private:
			virtual void onInit();
			
			bool Server_ok(awr1843aop::sensorCLI::Request &rq, awr1843aop::sensorCLI::Response &rp);

			ros::ServiceServer sensorSrv;

			std::string myUserPort;

			int myBaudRate;
	
	}; // Class Server
} // namespace awr1843aop
#endif
