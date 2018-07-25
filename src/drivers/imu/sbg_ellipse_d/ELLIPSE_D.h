/*
 * ELLIPSE_D.h
 *
 *  Created on: 18.07.2018
 *      Author: norman
 */

#ifndef DRIVERS_IMU_SBG_ELLIPSE_D_ELLIPSE_D_H_
#define DRIVERS_IMU_SBG_ELLIPSE_D_ELLIPSE_D_H_

#pragma once
#define PI 3.14159265359f
//----------------------------------------------------------------------//
//- Global definitions                                                 -//
//----------------------------------------------------------------------//
#define SBG_ECOM_MAX_BUFFER_SIZE				(512)							/*!< Maximum reception buffer size in bytes. */
#define SBG_ECOM_MAX_PAYLOAD_SIZE				(502)							/*!< Maximum payload size in bytes. */
#define SBG_ECOM_SYNC_1							(0xFF)							/*!< First synchronization char of the frame. */
#define SBG_ECOM_SYNC_2							(0x5A)							/*!< Second synchronization char of the frame. */
#define SBG_ECOM_ETX							(0x33)							/*!< End of frame byte. */

enum ELLIPSE_D_PARSE_STATE {
	ELLIPSE_D_PARSE_STATE0_SYNC1 = 0,
	ELLIPSE_D_PARSE_STATE1_SYNC2,
	ELLIPSE_D_PARSE_STATE2_MSG,
	ELLIPSE_D_PARSE_STATE3_CLASS,
	ELLIPSE_D_PARSE_STATE4_LEN1,
	ELLIPSE_D_PARSE_STATE5_LEN2,
	ELLIPSE_D_PARSE_STATE6_DATA,
	ELLIPSE_D_PARSE_STATE7_CRC1,
	ELLIPSE_D_PARSE_STATE8_CRC2,
	ELLIPSE_D_PARSE_STATE9_ETX
};

typedef enum _EllipseMsgStatus{
	MSG_EMPTY = 0,
	MSG_COMPLETE
} EllipseMsgStatus;

typedef struct{
	uint8_t id;
	uint8_t cl;
	uint16_t length;
	uint8_t data[SBG_ECOM_MAX_PAYLOAD_SIZE];
	uint16_t crc;
	EllipseMsgStatus status;
} ELLIPSE_MESSAGE;

/*!
 * Enum that defines all the message classes available.
 */
typedef enum _SbgEComClass
{
	SBG_ECOM_CLASS_LOG_ECOM_0			= 0x00,			/*!< Class that contains sbgECom protocol input/output log messages. */

	SBG_ECOM_CLASS_LOG_ECOM_1			= 0x01,			/*!< Class that contains special sbgECom output messages that handle high frequency output */

	SBG_ECOM_CLASS_LOG_NMEA_0			= 0x02,			/*!< Class that contains NMEA (and NMEA like) output logs. <br>
															 Note: This class is only used for identification purpose and does not contain any sbgECom message. */
	SBG_ECOM_CLASS_LOG_NMEA_1			= 0x03,			/*!< Class that contains proprietary NMEA (and NMEA like) output logs. <br>
															 Note: This class is only used for identification purpose and does not contain any sbgECom message. */
	SBG_ECOM_CLASS_LOG_THIRD_PARTY_0	= 0x04,			/*!< Class that contains third party output logs.
															Note: This class is only used for identification purpose and does not contain any sbgECom message. */
	SBG_ECOM_CLASS_LOG_CMD_0			= 0x10			/*!< Class that contains sbgECom protocol commands */
} SbgEComClass;

//----------------------------------------------------------------------//
//- Definition of all messages id for sbgECom                          -//
//----------------------------------------------------------------------//

/*!
 * Enum that defines all the available ECom output logs from the sbgECom library.
 */
typedef enum _SbgEComLog
{
	SBG_ECOM_LOG_STATUS 					= 1,		/*!< Status general, clock, com aiding, solution, heave */

	SBG_ECOM_LOG_UTC_TIME 					= 2,		/*!< Provides UTC time reference */

	SBG_ECOM_LOG_IMU_DATA 					= 3,		/*!< Includes IMU status, acc., gyro, temp delta speeds and delta angles values */

	SBG_ECOM_LOG_MAG 						= 4,		/*!< Magnetic data with associated accelerometer on each axis */
	SBG_ECOM_LOG_MAG_CALIB 					= 5,		/*!< Magnetometer calibration data (raw buffer) */

	SBG_ECOM_LOG_EKF_EULER 					= 6,		/*!< Includes roll, pitch, yaw and their accuracies on each axis */
	SBG_ECOM_LOG_EKF_QUAT 					= 7,		/*!< Includes the 4 quaternions values */
	SBG_ECOM_LOG_EKF_NAV 					= 8,		/*!< Position and velocities in NED coordinates with the accuracies on each axis */

	SBG_ECOM_LOG_SHIP_MOTION				= 9,		/*!< Heave, surge and sway and accelerations on each axis. */

	SBG_ECOM_LOG_GPS1_VEL 					= 13,		/*!< GPS velocities from primary or secondary GPS receiver */
	SBG_ECOM_LOG_GPS1_POS 					= 14,		/*!< GPS positions from primary or secondary GPS receiver */
	SBG_ECOM_LOG_GPS1_HDT 					= 15,		/*!< GPS true heading from dual antenna system */
	SBG_ECOM_LOG_GPS1_RAW					= 31,		/*!< GPS 1 raw data for post processing. */

	SBG_ECOM_LOG_GPS2_VEL					= 16,		/*!< GPS 2 velocity log data. */
	SBG_ECOM_LOG_GPS2_POS					= 17,		/*!< GPS 2 position log data. */
	SBG_ECOM_LOG_GPS2_HDT					= 18,		/*!< GPS 2 true heading log data. */
	SBG_ECOM_LOG_GPS2_RAW					= 38,		/*!< GPS 2 raw data for post processing. */

	SBG_ECOM_LOG_ODO_VEL 					= 19,		/*!< Provides odometer velocity */

	SBG_ECOM_LOG_EVENT_A 					= 24,		/*!< Event markers sent when events are detected on sync in A pin */
	SBG_ECOM_LOG_EVENT_B 					= 25,		/*!< Event markers sent when events are detected on sync in B pin */
	SBG_ECOM_LOG_EVENT_C					= 26,		/*!< Event markers sent when events are detected on sync in C pin */
	SBG_ECOM_LOG_EVENT_D 					= 27,		/*!< Event markers sent when events are detected on sync in D pin */
	SBG_ECOM_LOG_EVENT_E					= 28,		/*!< Event markers sent when events are detected on sync in E pin */

	SBG_ECOM_LOG_DVL_BOTTOM_TRACK			= 29,		/*!< Doppler Velocity Log for bottom tracking data. */
	SBG_ECOM_LOG_DVL_WATER_TRACK			= 30,		/*!< Doppler Velocity log for water layer data. */

	SBG_ECOM_LOG_SHIP_MOTION_HP				= 32,		/*!< Return delayed ship motion such as surge, sway, heave. */

	SBG_ECOM_LOG_PRESSURE					= 36,		/*!< Pressure sensor such as depth sensor or altimeter. */

	SBG_ECOM_LOG_USBL						= 37,		/*!< Raw USBL position data for subsea navigation. */

	SBG_ECOM_LOG_DEBUG_0					= 39,		/*!< Debug Log. */
	SBG_ECOM_LOG_IMU_RAW_DATA				= 40,		/*!< Factory only log. */
	SBG_ECOM_LOG_DEBUG_1					= 41,		/*!< Debug Log. */
	SBG_ECOM_LOG_DEBUG_2					= 42,		/*!< Debug Log. */
	SBG_ECOM_LOG_DEBUG_3					= 43,		/*!< Debug Log. */

	SBG_ECOM_LOG_ECOM_NUM_MESSAGES						/*!< Helper definition to know the number of ECom messages */
} SbgEComLog;

int ellipse_d_parser(char c, char *parserbuf, unsigned *parserbuf_index, enum ELLIPSE_D_PARSE_STATE *state, ELLIPSE_MESSAGE *msg);

/**
 * @brief Calculate CRC checksum for the specified buffer according to the polynom given in the IG500N protocol specification
 * @param pBuffer Pointer to the buffer
 * @param bufferSize Number of bytes in the buffer
 * @retval crc Calculated CRC checksum
 */
uint16_t calcCRC(const char *pBuffer, uint16_t bufferSize);


#endif /* DRIVERS_IMU_SBG_ELLIPSE_D_ELLIPSE_D_H_ */
