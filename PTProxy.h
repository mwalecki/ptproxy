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
#include <iostream>

#define DEFAULT_NFV2_BAUD B57600
#define COMM_BUFSZ 256
#define COMMAND_ARRAY_SZ 16

const float xJointsToMotors	10000;
const float yJointsToMotors	10000;

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
	void setMotorSpeed(float sx, float sy);

	/*!
	 *
	 */
	void setJointSpeed(float sx, float sy);

	/*!
	 *
	 */
	void setJointPos(float px, float py);

	/*!
	 *
	 */
	void getJointSpeed(float & sx, float & sy);

	/*!
	 *
	 */
	void getJointPos(float & px, float & py);


private:
	NF_STRUCT_ComBuf NFComBuf;
	SerialComm *CommPort;
	std::string portName;
	uint8_t rxBuf[COMM_BUFSZ];
	uint8_t txBuf[COMM_BUFSZ];
	uint8_t commandArray[COMMAND_ARRAY_SZ];
	uint8_t rxCnt, txCnt, commandCnt;

	float dx, dy;
	int32_t currentPosX, currentPosY;


};

#endif /* PTPROXY_H_ */
