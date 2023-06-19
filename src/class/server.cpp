#include "awr1843aop/server.hpp"

namespace awr1843aop
{
	PLUGINLIB_EXPORT_CLASS(awr1843aop::Server, nodelet::Nodelet);

	Server::Server() {}

	void Server::onInit()
	{
		ros::NodeHandle private_nh = getPrivateNodeHandle();
		ros::NodeHandle private_nh2("~"); // follow namespace for multiple sensors

		private_nh2.getParam("user_port", myUserPort);
		private_nh2.getParam("user_rate", myBaudRate);

		ROS_INFO("user_port = %s", myUserPort.c_str());
		ROS_INFO("user_rate = %d", myBaudRate);

		sensorSrv = private_nh.advertiseService("/sensorCLI", &Server::Server_ok, this);

		NODELET_DEBUG("Finished onInit function in Server");
	}

	bool Server::Server_ok(sensorCLI::Request &rq, sensorCLI::Response &rp)
	{
		NODELET_DEBUG("Port is \"%s\" and baud rate is %d", myUserPort.c_str(), myBaudRate);

		/* Open Serial port and error check */
		serial::Serial mySerialObj("", myBaudRate, serial::Timeout::simpleTimeout(1000)); // unit [ms]
		mySerialObj.setPort(myUserPort.c_str()); // reset "" to serial port path e.g., /dev/ttyUSB0

		// To safely open a serial port, how to respond if the serial port is already opened by another process
		try {
			mySerialObj.open();
		} 
		catch (std::exception &e1) {
			ROS_INFO("Failed to open User serial port with error: %s", e1.what());
			ROS_INFO("Waiting 20 seconds before trying again...");
			try {
				// Wait 20 seconds and trye to open serial port again
				ros::Duration(20).sleep();
				mySerialObj.open();
			}
			catch (std::exception &e2) {
				ROS_ERROR("Failed second time to open User serial port, error: %s", e2.what());
				NODELET_ERROR("Port could not be opened. Port is \"%s\" and baud rate is %d", myUserPort.c_str(), myBaudRate);
				return false;
			}
		}

		/* Read any previous pending response(s) */
		while (mySerialObj.available() > 0) {
			
			// arg1: save to "rp", arg2: max string len to read at once, arg3: string representing the end of the data
			// ":/>" string depends on protocal in serial port.
			mySerialObj.readline(rp.resp, 1024, ":/>");
			ROS_INFO("Received (previous) response from sensor: '%s'", rp.resp.c_str());
			rp.resp = "";
		}

		/* Send out command received from the clinet to awr1843aop sensor */
		ROS_INFO("Sending command to sensor: '%s'", rq.req.c_str());
		rq.req.append("\n");
		int bytesData = mySerialObj.write(rq.req);
		
		/* Read output from awr1843aop sensor */
		mySerialObj.readline(rp.resp, 1024, ":/>");
		ROS_INFO("Received response from sensor: '%s'", rp.resp.c_str());
		
		mySerialObj.close();

		return true;
	}
}


