#ifndef PARAM_PARSER_
#define PARAM_PARSER_

/* Include ROS headers */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

/* Include standard C/C++ headers */
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <vector>

/* Include awr1843aop package custom headers */
#include "awr1843aop/sensorCLI.h"


namespace awr1843aop
{
    class ParameterParser : public nodelet::Nodelet
    {
        public:
            ParameterParser();
            void ParamsParser(awr1843aop::sensorCLI &srv, ros::NodeHandle &nh);
            void CalParams(ros::NodeHandle &nh);

        private:
            virtual void onInit();
            awr1843aop::sensorCLI srv;
    }; // class ParameterParser
} // namespace awr1843aop
#endif
