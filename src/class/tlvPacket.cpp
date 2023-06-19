#include "awr1843aop/tlvPacket.hpp"


tlvPacket::tlvPacket(ros::NodeHandle *nh) : currentBufPtr(&buffers[0]), nextBufPtr(&buffers[1])
{
	tlvPacket_pub = nh->advertise<sensor_msgs::PointCloud2>("/awr1843aop/points_pcl", 100);
	points_pub    = nh->advertise<awr1843aop::points>("/awr1843aop/points", 100);
	marker_pub    = nh->advertise<visualization_msgs::Marker>("/awr1843aop/points_markers", 100);
    radarScan_pub = nh->advertise<awr1843aop::ScanPoints>("/awr1843aop/scan_points", 100);
	
	maxElevationAngleDeg = 90;
	maxAzimuthAngleDeg    = 90;
	
	// Wait for parameters
	while (!nh->hasParam("/awr1843aop/doppler_vel_resolution")) {}
	
	nh->getParam("/awr1843aop/numAdcSamples", nAdc);
	nh->getParam("/awr1843aop/numLoops", nLoop);
	nh->getParam("/awr1843aop/num_TX", nTx);
	nh->getParam("/awr1843aop/f_s", fs);	
	nh->getParam("/awr1843aop/f_c", fc);
	nh->getParam("/awr1843aop/BW", BW);
	nh->getParam("/awr1843aop/PRI", PRI);
	nh->getParam("/awr1843aop/t_fr", tfr);
	nh->getParam("/awr1843aop/max_range", rMax);
	nh->getParam("/awr1843aop/range_resolution", rRes);
	nh->getParam("/awr1843aop/max_doppler_vel", vMax);
	nh->getParam("/awr1843aop/doppler_vel_resolution", vRes);

	ROS_INFO("\n\n==============================\nList of parameters\n==============================\nNumber of range samples: %d\nNumber of chirps: %d\nf_s: %.3f MHz\nf_c: %.3f GHz\nBandwidth: %.3f MHz\nPRI: %.3f us\nFrame time: %.3f ms\nMax range: %.3f m\nRange resolution: %.3f m\nMax Doppler: +-%.3f m/s\nDoppler resolution: %.3f m/s\n==============================\n", \
        nAdc, nLoop, fs/1e6, fc/1e9, BW/1e6, PRI*1e6, tfr*1e3, rMax, rRes, vMax/2, vRes);
}

void tlvPacket::setFrameID(char* myFrameID)
{
	frameID = myFrameID;
}

void tlvPacket::setUARTPort(char* myDataPort)
{
	dataPort = myDataPort;
}

void tlvPacket::setBaudRate(int myBaudRate)
{
	dataBaudRate = myBaudRate;
}

void tlvPacket::setMaxElevationAngleDeg(int myMaxElevationAngleDeg)
{
	maxElevationAngleDeg = myMaxElevationAngleDeg;
}

void tlvPacket::setMaxAzimuthAngleDeg(int myMaxAzimuthAngleDeg)
{
	maxAzimuthAngleDeg = myMaxAzimuthAngleDeg;
}

//void setNodeHandle(ros::NodeHandle* nh)


/* @brief Implementation of Read incoming UART data thread */
void *tlvPacket::readIncomingData(void)
{
	int firstPacketReady  = 0;
	uint8_t last8Bytes[8] = {0};

	/* Open UART PORT and error checking */
	serial::Serial mySerialObj("", dataBaudRate, serial::Timeout::simpleTimeout(100)); // unit: [ms]
	mySerialObj.setPort(dataPort);
	try {
		mySerialObj.open();
	}
	catch (std::exception &e1) {
		ROS_INFO("Failed to open Data serial port with error: %s", e1.what());
		ROS_INFO("Waiting 20 seconds before trying again...");
	
		try {
			// Wait 20 seconds and try to open serial port again
			ros::Duration(20).sleep();
			mySerialObj.open();
		}
		catch (std::exception &e2) {
			ROS_ERROR("Failed second time to open Data serial port, error: %s", e2.what());
			ROS_ERROR("Port could not be opened. Port is \"%s\" and baud rate is %d", dataPort, dataBaudRate);
			pthread_exit(NULL);
		}
	}
	
	if (mySerialObj.isOpen())
		ROS_INFO("tlvPacket Read Thread: Port is open");
	else
		ROS_INFO("tlvPacket Read Thread: Port could not be opened");
	
	/* @brief Quick magicword check to synchronize program with data stream */
	while (!isMagicWord(last8Bytes)) {
		last8Bytes[0] = last8Bytes[1];
		last8Bytes[1] = last8Bytes[2];
		last8Bytes[2] = last8Bytes[3];
		last8Bytes[3] = last8Bytes[4];
		last8Bytes[4] = last8Bytes[5];
		last8Bytes[5] = last8Bytes[6];
		last8Bytes[6] = last8Bytes[7];
		mySerialObj.read(&last8Bytes[7], 1);
    }
	
    /* @brief Lock nextBufPtr before entering main loop */
    pthread_mutex_lock(&nextBufPtr_mutex);
    
    while (ros::ok()) {
        
        /* Start reading UART data and writing to buffer while also checking for magicWord */
        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObj.read(&last8Bytes[7], 1);

        nextBufPtr->push_back( last8Bytes[7]);   // push byte onto buffer

		/* If a magicword is found wait for sorting to finish and switch buffers */
		if (isMagicWord(last8Bytes)) {

			/* Lock countSync Mutex while unlocking nextBufPtr so that the sawp thread can use it */
			pthread_mutex_lock(&countSync_mutex);
			pthread_mutex_unlock(&nextBufPtr_mutex);

			/* increment countSync */
			++countSync;
	
			/* If this is the first packet to be found, increment countSync again since Sort thread is not reading data yet */
			if (firstPacketReady == 0) {
				++countSync;
				firstPacketReady = 1;
			}

			/* Signal Swap Thread to run if countSync has reached its max value */
			if (countSync == COUNT_SYNC_MAX) {
				pthread_cond_signal(&countSync_max_cv);
			}

			/* Wait for the Swap thread to finish swapping pointers and signal us to continue */
			pthread_cond_wait(&read_go_cv, &countSync_mutex);
		
			/* Unlock countSync so that Swap Thread can use it */
			pthread_mutex_unlock(&countSync_mutex);
			pthread_mutex_lock(&nextBufPtr_mutex);

			nextBufPtr->clear();
			std::memset(last8Bytes, 0, sizeof(last8Bytes));
		}
	}
	
	mySerialObj.close();
	pthread_exit(NULL);
}


int tlvPacket::isMagicWord(uint8_t last8Bytes[8])
{
	int val(0), i(0), j(0);
	
	for (i=0; i<8; ++i) {
		if (last8Bytes[i] == magicWord[i]) {
			++j;
		}
	}
	
	if (j==8)
		val = 1;
	
	return val;
}


/* 0x0102 => in stack (0x02|0x01) => swap to (0x01|0x02) */
/* @brief 0x01 => 16x16 = 2^(4+4) = 1 [byte] */
void *tlvPacket::syncedBufferSwap(void)
{
	while (ros::ok())
	{
		pthread_mutex_lock(&countSync_mutex);
		
		while (countSync < COUNT_SYNC_MAX)
		{
			pthread_cond_wait(&countSync_max_cv, &countSync_mutex);
			pthread_mutex_lock(&currentBufPtr_mutex);
			pthread_mutex_lock(&nextBufPtr_mutex);
			
			std::vector<uint8_t>* tempBufPtr = currentBufPtr;
			this->currentBufPtr = this->nextBufPtr;
			this->nextBufPtr = tempBufPtr;
			
			pthread_mutex_unlock(&currentBufPtr_mutex);
			pthread_mutex_unlock(&nextBufPtr_mutex);
			
			countSync = 0;
			
			pthread_cond_signal(&sort_go_cv);
			pthread_cond_signal(&read_go_cv);
		}
		pthread_mutex_unlock(&countSync_mutex);
	}
	pthread_exit(NULL);
}


void *tlvPacket::sortIncomingData(void)
{
	mmWave_Output_TLV_Types tlvType = MMWAVE_OUTPUT_NULL;
	uint32_t tlvLen = 0;
	uint32_t headerSize;
	unsigned int currentDataPosition = 0;
	SorterState sorterState = READ_HEADER;
	int i(0), j(0), tlvCount(0), offset(0);
	float maxElevationAngleRatioSquared;
	float maxAzimuthAngleRatio;
	
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RScan(new pcl::PointCloud<pcl::PointXYZI>);
	awr1843aop::points radarPoints;
    std::shared_ptr<awr1843aop::ScanPoints> ScanPtr(new awr1843aop::ScanPoints);

	// wait for first packet to arrive
	pthread_mutex_lock(&countSync_mutex);
	pthread_cond_wait(&sort_go_cv, &countSync_mutex);
	pthread_mutex_unlock(&countSync_mutex);
	
	pthread_mutex_lock(&currentBufPtr_mutex);
	while (ros::ok())
	{
		switch (sorterState)
		{
			case READ_HEADER:
				
				// init variables
				mmwData.numObjOut = 0;
				
				// make sure packet has at least first three fields (12 bytes) before we read them (does not include magicWord since it was already removed)
                if (currentBufPtr->size() < 12) {
					sorterState = SWAP_BUFFERS;
					break;
				}
				
				// get version (4bytes)
                std::memcpy( &mmwData.header.version, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.header.version));
				currentDataPosition += (sizeof(mmwData.header.version));

				// get total packet length (4bytes)
                std::memcpy( &mmwData.header.totalPacketLen, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.header.totalPacketLen));
				currentDataPosition += (sizeof(mmwData.header.totalPacketLen));
				
                // get platform (4bytes)
                std::memcpy( &mmwData.header.platform, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.header.platform));
				currentDataPosition += (sizeof(mmwData.header.platform));
		
				// if packet doesn't have correct header size (which is based on platform), throw it away
				// platfrom is awr1843aop
				headerSize = 8 * 4;
				if (currentBufPtr->size() < headerSize) {
					sorterState = SWAP_BUFFERS;
					break;
				}
				
				// get frameNumber (4bytes)
                std::memcpy( &mmwData.header.frameNumber, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.header.frameNumber));
				currentDataPosition += (sizeof(mmwData.header.frameNumber));
				// get timeCpuCycles (4bytes)
                std::memcpy( &mmwData.header.timeCpuCycles, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.header.timeCpuCycles));
				currentDataPosition += (sizeof(mmwData.header.timeCpuCycles));

				// get numDetectedObj (4bytes)
                std::memcpy( &mmwData.header.numDetectedObj, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.header.numDetectedObj));
				currentDataPosition += (sizeof(mmwData.header.numDetectedObj));
	
				// get numTLVs (4bytes)
                std::memcpy( &mmwData.header.numTLVs, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.header.numTLVs));
				currentDataPosition += (sizeof(mmwData.header.numTLVs));

				// get subFrameNumber (4bytes)
                std::memcpy( &mmwData.header.subFrameNumber, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.header.subFrameNumber));
				currentDataPosition += (sizeof(mmwData.header.subFrameNumber));

				// if packet lengths do not match, throw it away
				if (mmwData.header.totalPacketLen == currentBufPtr->size()) {
					sorterState = CHECK_TLV_TYPE;
				}
				else 
					sorterState = SWAP_BUFFERS;
				break;
	

			case READ_OBJ_STRUCT:
				// CHECK_TLV_TYPE case has already read tlvType and tlvLen

				i = 0;
				offset = 0;
				// SDK version is 3.x or newer
                if (((mmwData.header.version >> 24) & 0xFF) >= 3) {
					//ROS_INFO("SDK version is 3.x or newer");
					mmwData.numObjOut = mmwData.header.numDetectedObj;
				}

				RScan->header.frame_id = frameID;
                RScan->header.stamp = ros::Time::now().toNSec()/1e3;
				RScan->height = 1;
				RScan->width = mmwData.numObjOut;
				RScan->is_dense = 1;
				RScan->points.resize(RScan->width * RScan->height);

				// Calculate ratios for max desired elevation and azimuth angles
				if ((maxElevationAngleDeg >= 0) && (maxElevationAngleDeg < 90)) {
					maxElevationAngleRatioSquared = tan(maxElevationAngleDeg * M_PI / 180.0);
					maxElevationAngleRatioSquared = maxElevationAngleRatioSquared * maxElevationAngleRatioSquared;
				}
				else 
					maxElevationAngleRatioSquared = -1;
				
				if ((maxAzimuthAngleDeg >=0) && (maxAzimuthAngleDeg < 90)) {
					maxAzimuthAngleRatio = tan(maxAzimuthAngleDeg * M_PI / 180.0);
				}
				else
					maxAzimuthAngleDeg = -1;

                // When mmwData.numObjOut != 0
                if (mmwData.numObjOut != 0) {
                    ScanPtr->header.frame_id = frameID;
                    ScanPtr->header.stamp = ros::Time::now();
                    ScanPtr->total_points = mmwData.numObjOut;
                }
                //else
			
				// Populate pointcloud
				while (i < mmwData.numObjOut) {
					if (((mmwData.header.version >> 24) & 0xFF) >= 3) { // SDK version is 3.x or newer
						//get object x-coordinate [meters]
                        std::memcpy( &mmwData.newObjOut.x, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.newObjOut.x));
						currentDataPosition += (sizeof(mmwData.newObjOut.x));
						
						// get object y-coordinate [meters]
                        std::memcpy( &mmwData.newObjOut.y, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.newObjOut.y));
						currentDataPosition += (sizeof(mmwData.newObjOut.y));

						// get object z-coordinate [meters]
                        std::memcpy( &mmwData.newObjOut.z, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.newObjOut.z));
						currentDataPosition += (sizeof(mmwData.newObjOut.z));

						// get object radial velocity [m/s]
                        std::memcpy( &mmwData.newObjOut.velocity, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.newObjOut.velocity));
						currentDataPosition += (sizeof(mmwData.newObjOut.velocity));

						// Map awr1843aop sensor coordinates to ROS coordinate system
						RScan->points[i].x = mmwData.newObjOut.y;   // ROS standard coordinate system X-axis is forward which is the awr1843aop sensor Y-axis
						RScan->points[i].y = -mmwData.newObjOut.x;   // ROS standard coordinate system Y-axis is left which is the awr1843aop sensor -(X-axis)
						RScan->points[i].z = mmwData.newObjOut.z;   // ROS standard coordinate system Z-axis is up which is the same as awr1843aop sensor Z-axis

						radarPoints.header.frame_id = frameID;
						radarPoints.header.stamp = ros::Time::now();
						radarPoints.point_id = i;
						radarPoints.x = mmwData.newObjOut.y;
						radarPoints.y = -mmwData.newObjOut.x;
						radarPoints.z = mmwData.newObjOut.z;
						radarPoints.velocity = mmwData.newObjOut.velocity;
						// For SDK 3.x, intensity is replaced by snr in sideInfo and is parsed in the READ_SIDE_INFO case
                        ScanPtr->point_id = i;
                        ScanPtr->x = mmwData.newObjOut.x;
                        ScanPtr->y = mmwData.newObjOut.y;
                        ScanPtr->z = mmwData.newObjOut.z;
                        ScanPtr->velocity = mmwData.newObjOut.velocity;
                    }

					if ((maxElevationAngleRatioSquared == -1) || (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x *RScan->points[i].x + RScan->points[i].y * RScan->points[i].y)) < maxElevationAngleRatioSquared) &&
						((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) && (RScan->points[i].x != 0))
					{
						points_pub.publish(radarPoints);
                        radarScan_pub.publish(*ScanPtr);
					}
					i++;
				}
		
			sorterState = CHECK_TLV_TYPE;
			break;
		
		
		case READ_SIDE_INFO:
			
			// Make sure we already received and parsed detected obj list (READ_OBJ_STRUCT)
			if (mmwData.numObjOut > 0) {
				for (i=0; i < mmwData.numObjOut; i++) {
					// get snr [unit is 0.1 steps of dB]
					std::memcpy( &mmwData.sideInfo.snr, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.sideInfo.snr));
					currentDataPosition += (sizeof(mmwData.sideInfo.snr));

					// get noise [unit is 0.1 steps of dB]
					std::memcpy( &mmwData.sideInfo.noise, &currentBufPtr->at(currentDataPosition), sizeof(mmwData.sideInfo.noise));
					currentDataPosition += (sizeof(mmwData.sideInfo.noise));

					RScan->points[i].intensity = (float) mmwData.sideInfo.snr / 10.0;    // Use snr for "intensity" field (divide by 10 since unit of snr is 0.1dB)
                    ScanPtr->snr = (float) mmwData.sideInfo.snr / 10.0;
                }
			}
			else // else just skip side info section if we have not already received and parsed detected obj list
            {
				i = 0;	
				currentDataPosition += tlvLen;
			}
			
			sorterState = CHECK_TLV_TYPE;
			break;
	

		case READ_LOG_MAG_RANGE:
			//i=0;
			//currentDataPosition += tlvLen;
			sorterState = CHECK_TLV_TYPE;
			break;

		case READ_NOISE:
			i=0;
			currentDataPosition += tlvLen;
			sorterState = CHECK_TLV_TYPE;
			break;

		case READ_AZIMUTH:
			i=0;
			currentDataPosition += tlvLen;
			sorterState = CHECK_TLV_TYPE;
			break;
		
		case READ_DOPPLER:
			i=0;
			currentDataPosition += tlvLen;
			sorterState = CHECK_TLV_TYPE;
			break;

		case READ_STATS:
			i=0;
			currentDataPosition += tlvLen;
			sorterState = CHECK_TLV_TYPE;
			break;
		
		case CHECK_TLV_TYPE:
			
			if (tlvCount++ >= mmwData.header.numTLVs) // Done parsing all recieved TLV sections
			{
				// Publish detected object pointcloud
				if (mmwData.numObjOut > 0)
				{
					j = 0;
					for (i=0; i < mmwData.numObjOut; i++)
					{
						// Keep point if elevation and azimuth angles are less than specified max values
						// (NOTE: The following calculations are done using ROS standard coordinates system axis definitions where X is forward and Y is left)
                        if (((maxElevationAngleRatioSquared == -1) ||
                             (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
                                                                            RScan->points[i].y * RScan->points[i].y)
                              ) < maxElevationAngleRatioSquared)
                            ) &&
                            ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
                                    (RScan->points[i].x != 0)
                           )
                        {
                            std::memcpy( &RScan->points[j], &RScan->points[i], sizeof(RScan->points[i]));
                            ++j;
                        }
                    }
                
                    mmwData.numObjOut = j;   // update number of objects as some points may have been removed

                    // Resize point cloud since some points may have been removed
                    RScan->width = mmwData.numObjOut;
                    RScan->points.resize(RScan->width * RScan->height);
                   
                    tlvPacket_pub.publish(RScan);
                }
                sorterState = SWAP_BUFFERS;
            }
            else // More TLV sections to parse
            {
                // get tlvType (32 bits) & remove from queue
                std::memcpy( &tlvType, &currentBufPtr->at(currentDataPosition), sizeof(tlvType));
                currentDataPosition += (sizeof(tlvType));

                // get tlvLen (32 bits) & remove from queue
                std::memcpy( &tlvLen, &currentBufPtr->at(currentDataPosition), sizeof(tlvLen));
                currentDataPosition += (sizeof(tlvLen));

                switch(tlvType)
                {
                    case MMWAVE_OUTPUT_DETECTED_POINTS:
                        sorterState = READ_OBJ_STRUCT;
                        break;

                    case MMWAVE_OUTPUT_RANGE_PROFILE:
                        sorterState = READ_LOG_MAG_RANGE;
                        break;

                    case MMWAVE_OUTPUT_NOISE_PROFILE:
                        sorterState = READ_NOISE;
                        break;

                    case MMWDEMO_OUTPUT_AZIMUTH_STATIC_HEAT_MAP:
                        sorterState = READ_AZIMUTH;
                        break;
                    
                    case MMWDEMO_OUTPUT_RANGE_DOPPLER_HEAT_MAP:
                        sorterState = READ_DOPPLER;
                        break;

                    case MMWDEMO_OUTPUT_MSG_STATS:
                        sorterState = READ_STATS;
                        break;

                    case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
                        sorterState = READ_SIDE_INFO;
                        break;

                    case MMWDEMO_OUTPUT_MSG_MAX:
                        sorterState = READ_HEADER;
                        break;

                    default:
                        break;
                }
            }
            break;

        case SWAP_BUFFERS:

            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&currentBufPtr_mutex);
            ++countSync;

            if (countSync == COUNT_SYNC_MAX) {
                pthread_cond_signal(&countSync_max_cv);
            }
            pthread_cond_wait(&sort_go_cv, &countSync_mutex);

            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&currentBufPtr_mutex);

            currentDataPosition = 0;
            tlvCount = 0;
            sorterState = READ_HEADER;
            break;
        
        default:
            break;
        }
    }
    pthread_exit(NULL);
}


void tlvPacket::start(void)
{	
	pthread_t uartThread, sorterThread, swapThread;
	
	pthread_mutex_init(&countSync_mutex, NULL);
	pthread_mutex_init(&nextBufPtr_mutex, NULL);
	pthread_mutex_init(&currentBufPtr_mutex, NULL);
	pthread_cond_init(&countSync_max_cv, NULL);
	pthread_cond_init(&read_go_cv, NULL);
	pthread_cond_init(&sort_go_cv, NULL);

	int isThreadZero1, isThreadZero2, isThreadZero3;
	countSync = 0;

	/* Create independent threads each of which will execute function */
	isThreadZero1 = pthread_create( &uartThread, NULL, this->readIncomingData_helper, this);
	if (isThreadZero1) {
		ROS_INFO("Error: pthread_create() return code: %d\n", isThreadZero1);
		ros::shutdown();
	}

	isThreadZero2 = pthread_create( &sorterThread, NULL, this->sortIncomingData_helper, this);
	if (isThreadZero2) {
		ROS_INFO("Error: pthread_create() return code: %d\n", isThreadZero2);
		ros::shutdown();
	}
	
	isThreadZero3 = pthread_create( &swapThread, NULL, this->syncedBufferSwap_helper, this);
	if (isThreadZero3) {
		ROS_INFO("Error: pthread_create() return code: %d\n", isThreadZero3);
		ros::shutdown();
	}
	
	ros::spin();
	
	/* Wait until threads terminated */
	pthread_join(isThreadZero1, NULL);
	ROS_INFO("tlvPacket Read Thread joined");
	pthread_join(isThreadZero2, NULL);
	ROS_INFO("tlvPacket Sort Thread joined");
	pthread_join(isThreadZero3, NULL);
	ROS_INFO("tlvPacket Swap Thread joined");
	
	pthread_mutex_destroy(&countSync_mutex);
	pthread_mutex_destroy(&nextBufPtr_mutex);
	pthread_mutex_destroy(&currentBufPtr_mutex);
	pthread_cond_destroy(&countSync_max_cv);
	pthread_cond_destroy(&read_go_cv);
	pthread_cond_destroy(&sort_go_cv);
}
		
void *tlvPacket::readIncomingData_helper(void *context)
{
	return (static_cast<tlvPacket*>(context)->readIncomingData());
}
	
void *tlvPacket::sortIncomingData_helper(void *context)
{
	return (static_cast<tlvPacket*>(context)->sortIncomingData());
}

void *tlvPacket::syncedBufferSwap_helper(void *context)
{
	return (static_cast<tlvPacket*>(context)->syncedBufferSwap());
}


/* Visualization Function */
void tlvPacket::visualize(const awr1843aop::points &msg)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = frameID;
	marker.header.stamp = ros::Time::now();
	marker.id = msg.point_id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = marker.ADD;
	
	marker.pose.position.x = msg.x;
	marker.pose.position.y = msg.y;
	marker.pose.position.z = msg.z;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 0;
	
	marker.scale.x = .03;
	marker.scale.y = .03;
	marker.scale.z = .03;

	marker.color.a = 1;   // transparent None
	marker.color.r = (int) 255 * msg.intensity; // normalizing
	marker.color.g = (int) 255 * msg.intensity; // normalizing
	marker.color.b = 1;
	
	marker_pub.publish(marker);
}
