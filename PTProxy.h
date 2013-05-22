/*
 * PTProxy.h
 *
 *  Created on: 29-11-2012
 *      Author: wacek
 */

#ifndef PTPROXY_H_
#define PTPROXY_H_

#include "nf/nfv2.h"
#include "serialcomm/serialcomm.hpp"
#include "can/MotorController.h"
#include <iostream>
#include <math.h>

#define DEFAULT_X_GEAR_RATIO	46
#define DEFAULT_Y_GEAR_RATIO	50
#define DEFAULT_X_ENCODER_RES	2000
#define DEFAULT_Y_ENCODER_RES	2000
#define DEFAULT_X_MOTOR_OFFSET	1707
#define DEFAULT_Y_MOTOR_OFFSET	-3139

#define DEFAULT_NFV2_BAUD B57600
#define COMM_BUFSZ 256
#define COMMAND_ARRAY_SZ 16

/*!
 *
 */
class PTProxy {
public:
	/*!
	 *
	 */
	PTProxy(std::string port);

	/*!
	 *
	 */
	virtual ~PTProxy();

	/*!
	 *
	 */
	bool isConnected() const;

	/*!
	 *
	 */
	void nextStep(void);

	/*!
	 *
	 */
	void startSynchronization();

	/*!
	 *
	 */
	bool isSynchronized();

	/*!
	 *
	 */
	int setMotorSpeed(float sx, float sy);

	/*!
	 *
	 */
	int setMotorPosition(float px, float py);

	/*!
	 *
	 */
	int setMotorPositionWithSpeed(float px, float py, float sx, float sy);

	/*!
	 *
	 */
	int setJointSpeed(float sx, float sy);

	/*!
	 *
	 */
	int setJointPosition(float px, float py);

	/*!
	 *
	 */
	int setJointPositionWithSpeed(float px, float py, float sx, float sy);

	/*!
	 *
	 */
	void getMotorSpeed(float & sx, float & sy);

	/*!
	 *
	 */
	void getMotorPosition(float & px, float & py);

	/*!
	 *
	 */
	void getJointSpeed(float & sx, float & sy);

	/*!
	 *
	 */
	void getJointPosition(float & px, float & py);

	/*!
	 *
	 */
	void setJointsToMotorsRatio(float jtmx, float jtmy);

	/*!
	 *
	 */
	void getJointsToMotorsRatio(float & jtmx, float & jtmy);

	/*!
	 *
	 */
	void setMotorsOffset(int offx, int offy);

	/*!
	 *
	 */
	void getMotorsOffset(int & offx, int & offy);

private:
	NF_STRUCT_ComBuf NFComBuf;
	SerialComm *CommPort;
	MotorController *can;
	std::string portName;
	uint8_t rxBuf[COMM_BUFSZ];
	uint8_t txBuf[COMM_BUFSZ];
	uint8_t commandArray[COMMAND_ARRAY_SZ];
	uint8_t rxCommandArray[COMMAND_ARRAY_SZ];
	uint8_t rxCnt, txCnt, commandCnt, rxCommandCnt;

	float xJointsToMotorsRatio;
	float yJointsToMotorsRatio;
	int xMotorsOffset;
	int yMotorsOffset;
};

#endif /* PTPROXY_H_ */
