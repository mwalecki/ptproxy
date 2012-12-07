/*
 * PTProxy.cpp
 *
 *  Created on: 29-11-2012
 *      Author: wacek
 */

#include "PTProxy.h"

PTProxy::PTProxy(std::string port) : portName(port) {
	NFv2_Config2(&NFComBuf, NF_MasterAddress, NF_AddressBase);
	NFv2_CrcInit();
	CommPort = new SerialComm(portName, DEFAULT_NFV2_BAUD);
	if (! (CommPort->isConnected()) ) {
		std::cout << "Connection failed!" << std::endl;
	}
	else{
		std::cout << "Connected to " << port << std::endl;
	}

	NFComBuf.SetDrivesMode.data[0] = NF_DrivesMode_SPEED;
	NFComBuf.SetDrivesMode.data[1] = NF_DrivesMode_SPEED;
	commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
}

PTProxy::~PTProxy() {
}

void PTProxy::nextStep(void){
//	NFComBuf.SetDrivesPosition.data[0] = dx;
//	NFComBuf.SetDrivesPosition.data[1] = dy;
//	commandArray[commandCnt++] = NF_COMMAND_SetDrivesPosition;
	NFComBuf.SetDrivesSpeed.data[0] = dx;
	NFComBuf.SetDrivesSpeed.data[1] = dy;
	commandArray[commandCnt++] = NF_COMMAND_SetDrivesSpeed;

	// If communication with requested
	if(commandCnt > 0) {
		txCnt = NF_MakeCommandFrame(&NFComBuf, txBuf, (const uint8_t*)commandArray, commandCnt, NF_AddressBase);
		// Clear communication request
		commandCnt = 0;
		// Send command frame to Elektron
		CommPort->write(txBuf, txCnt);
	}
}

bool PTProxy::isConnected() const {
	return (CommPort->isConnected());
}

void PTProxy::setMotorSpeed(float sx, float sy){
	dx = sx;
	dy = sy;
}

void PTProxy::setJointSpeed(float sx, float sy) {
}

void PTProxy::setJointPos(float px, float py) {
}

void PTProxy::getJointSpeed(float& sx, float& sy) {
}

void PTProxy::getJointPos(float& px, float& py) {
}
