#ifndef DATA_PORT
#define DATA_PORT

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

/* Include awr1843aop package custom headers */
#include "awr1843aop/tlvPacket.hpp"

namespace awr1843aop
{
	class DataPort : public nodelet::Nodelet
	{
		public:
			DataPort();

		private:
			virtual void onInit();

	}; // class DataPort
} // namespace awr1843aop
#endif
