#include "awr1843aop/DataPort.hpp"
#include "awr1843aop/tlvPacket.hpp"

namespace awr1843aop
{
	PLUGINLIB_EXPORT_CLASS(awr1843aop::DataPort, nodelet::Nodelet);
	
	DataPort::DataPort() {}

	void DataPort::onInit()
	{
		ros::NodeHandle private_nh("~");

		std::string myDataPort;
		std::string myFrameID;
		int myBaudRate;
		int myMaxElevationAngleDeg;
		int myMaxAzimuthAngleDeg;

		private_nh.getParam("data_port", myDataPort);
		private_nh.getParam("data_rate", myBaudRate);
		private_nh.getParam("frame_id", myFrameID);

		// Use max angle if none specified
		if (!(private_nh.getParam("max_elevation_angle_deg", myMaxElevationAngleDeg))) {
			myMaxElevationAngleDeg = 90;
		}
		if (!(private_nh.getParam("max_azimuth_angle_deg", myMaxAzimuthAngleDeg))) {
			myMaxAzimuthAngleDeg = 90;
		}

		ROS_INFO("data_port = %s", myDataPort.c_str());
		ROS_INFO("data_rate = %d", myBaudRate);
		ROS_INFO("max_elevation_angle_deg = %d", myMaxElevationAngleDeg);
		ROS_INFO("max_azimuth_angle_deg = %d", myMaxAzimuthAngleDeg);
		
		tlvPacket Packet(&private_nh);
		Packet.setFrameID( (char*)myFrameID.c_str());
		Packet.setUARTPort( (char*)myDataPort.c_str());
		Packet.setBaudRate(myBaudRate);
		Packet.setMaxElevationAngleDeg(myMaxElevationAngleDeg);
		Packet.setMaxAzimuthAngleDeg(myMaxAzimuthAngleDeg);
		Packet.start();

		NODELET_DEBUG("Finished onInit function of data in DataPort");
	}
}
