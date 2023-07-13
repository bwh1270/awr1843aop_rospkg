#ifndef TLV_PACKET
#define TLV_PACKET

/* Include ROS headers */
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "pcl_ros/point_cloud.h"
#include "visualization_msgs/Marker.h"

/* Include standard C/C++ headers */
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>
#include <memory> // C++11
#include <boost/shared_ptr.hpp>
#include <pthread.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/* Include awr1843aop package custom headers */
#include "awr1843aop/radarParams.hpp"
#include "awr1843aop/points.h"
#include "awr1843aop/radarPoints.h"


// Define Macros
#define COUNT_SYNC_MAX 2


class tlvPacket
{
	public:
		tlvPacket(ros::NodeHandle* nh);
		
		void setFrameID(char* myFrameID);
		void setUARTPort(char* myDataPort);
		void setBaudRate(int myBaudRate);
		void setMaxElevationAngleDeg(int myMaxElevationAngleDeg);
		void setMaxAzimuthAngleDeg(int myMaxAzimuthAngleDeg);
		void setNodeHandle(ros::NodeHandle* nh);

		/* User callable function to start the tlvPacket's internal threads */
		void start(void);

		/* Helper function to allow pthread compatability */
		static void* readIncomingData_helper(void *context);
		static void* sortIncomingData_helper(void *context);
		static void* syncedBufferSwap_helper(void *context);

		/* Sorted radarParams structure */
		mmwDataPacket mmwData;
	
	private:
		int nAdc;      // the number of ADC Sampling
		int nLoop;     // the number of Loops
		int nTx;       // the number of TX
		float fs;      // Digital sampling rate
		float fc;      // carier frequency
		float BW;      // BandWidth
		float PRI;     // idle time + rampEndTime
		float tfr;     // frame periodicity
		float rMax;    // maximum range
		float rRes;    // range resolution
		float vMax;    // maximum radial velocity
		float vRes;    // velocity resolution

		char* frameID;
		char* dataPort;
		int dataBaudRate;
		int maxElevationAngleDeg;
		int maxAzimuthAngleDeg;
		int countSync; // Mutex protected variable which synchronizes threads

		/* @brief Read/Write Buffers */
		std::vector<uint8_t> buffers[2];

		/* @brief Pointer to current data (sort) */
		std::vector<uint8_t> *currentBufPtr;

		/* @brief Pointer to new data (read) */
		std::vector<uint8_t> *nextBufPtr;

		/* @brief Mutex protecting the countSync variable */
		pthread_mutex_t countSync_mutex;

		/* @brief Mutex protecting the currentBufPtr pointer */
		pthread_mutex_t currentBufPtr_mutex;
		
		/* @brief Mutex protecting the nextBufPtr pointer */
		pthread_mutex_t nextBufPtr_mutex;

		/* @brief Condition variable which blocks the Swap Thread until signaled */
		pthread_cond_t countSync_max_cv;

		/* @brief Condition variable which blocks the Read Thread until signaled */
		pthread_cond_t read_go_cv;

		/* @brief Condition variable which blocks the Sort Thread until signaled */
		pthread_cond_t sort_go_cv;

		/* @brief Swap Buffer Pointers Thread */
		void *syncedBufferSwap(void);

		/* @brief Checks if the magic word was found */
		int isMagicWord(uint8_t last8Bytes[8]);

		/* @brief Read incoming UART Data Thread */
		void *readIncomingData(void);

		/* @brief Sort incoming UART Data Thread */
		void *sortIncomingData(void);

		void visualize(const awr1843aop::points &msg);

		ros::NodeHandle* nodeHandle;
		ros::Publisher tlvPacket_pub;
		ros::Publisher points_pub;
		ros::Publisher marker_pub;

        /* Customed MSG Publisher */
		ros::Publisher radarPoints_pub;
};
#endif
