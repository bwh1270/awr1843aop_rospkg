#ifndef _TI_DATA_STRUCTURES_
#define _TI_DATA_STRUCTURES_

#include <iostream>
#include <cstdio>
#include <cstdint>
#include <boost/thread.hpp>
#include <memory>

#include "ros/ros.h"
#include "serial/serial.h"


enum mmWave_Output_TLV_Types
{
    MMWAVE_OUTPUT_NULL = 0,

    /*! @brief List of detected points for the frame */
    MMWAVE_OUTPUT_DETECTED_POINTS,

    /*! @brief Range profile */
    MMWAVE_OUTPUT_RANGE_PROFILE,
    
    /*! @brief Noise floor profile */
    MMWAVE_OUTPUT_NOISE_PROFILE,

    /*! @brief   Is not for AOP */
    MMWDEMO_OUTPUT_AZIMUTH_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    MMWDEMO_OUTPUT_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Statistics(Performance) information */
    MMWDEMO_OUTPUT_MSG_STATS,

    /*! @brief   List of detected points side information */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO,

    MMWDEMO_OUTPUT_MSG_MAX
};

enum SorterState{ READ_HEADER, 
    CHECK_TLV_TYPE,
    READ_OBJ_STRUCT, 
    READ_LOG_MAG_RANGE, 
    READ_NOISE, 
    READ_AZIMUTH, 
    READ_DOPPLER, 
    READ_STATS,
    SWAP_BUFFERS,
    READ_SIDE_INFO};

struct mmWave_output_frame_header
{
    /*! @brief   Version: MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum */
    uint32_t    version;

    /*! @brief   Total packet length including header in Bytes */
    uint32_t    totalPacketLen;

    /*! @brief   platform type */
    uint32_t    platform;
    
    /*! @brief   Frame number */
    uint32_t    frameNumber;

    /*! @brief   Time in CPU cycles when the message was created. For XWR16xx: DSP CPU cycles, for XWR14xx: R4F CPU cycles */
    uint32_t    timeCpuCycles;
    
    /*! @brief   Number of detected objects */
    uint32_t    numDetectedObj;

    /*! @brief   Number of TLVs */
    uint32_t    numTLVs;

    /*! @brief   Sub-frame Number (not used with XWR14xx) */
    uint32_t    subFrameNumber;
};


typedef struct PointCloudCartesian_t
{
/*! @brief x - coordinate in meters */
float x;

/*! @brief y - coordinate in meters */
float y;

/*! @brief z - coordinate in meters */
float z;

/*! @brief Doppler velocity estimate in m/s. Positive velocity means target
* is moving away from the sensor and negative velocity means target
* is moving towards the sensor. */
float velocity;
}PointCloudCartesian_t;


typedef struct PointCloudSideInfo_t
{
/*! @brief snr - CFAR cell to side noise ratio in dB expressed in 0.1 steps of dB */
int16_t snr;

/*! @brief y - CFAR noise level of the side of the detected cell in dB expressed in 0.1 steps of dB */
int16_t noise;
}PointCloudSideInfo_t;


const uint8_t magicWord[8] = {2, 1, 4, 3, 6, 5, 8, 7};

struct mmwDataPacket{
mmWave_output_frame_header header;
uint16_t numObjOut;

PointCloudCartesian_t newObjOut; // used for SDK 3.x
PointCloudSideInfo_t sideInfo; // used for SDK 3.x
};

#endif
