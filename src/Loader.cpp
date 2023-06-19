#include "ros/ros.h"
#include "nodelet/loader.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "awr1843aop_Manager");

	nodelet::Loader manager(true);
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;

	manager.load("Server",   "awr1843aop/Server",   remap, nargv);
	manager.load("DataPort", "awr1843aop/DataPort", remap, nargv);

	ros::spin();
	return 0;
}
