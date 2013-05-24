/*
 * PTProxy.cpp
 *
 *  Created on: 29-11-2012
 *      Author: wacek
 */

#include "PTProxy.h"

PTProxy::PTProxy(std::string port) : portName(port) {
	if(port.find("can") == std::string::npos) {
		NFv2_Config2(&NFComBuf, NF_MasterAddress, NF_AddressBase);
		NFv2_CrcInit();
		CommPort = new SerialComm(portName, DEFAULT_NFV2_BAUD);
		if (! (CommPort->isConnected()) ) {
			std::cout << "Connection failed!" << std::endl;
		} else{
			std::cout << "Connected to " << port << std::endl;
		}
		rxCnt = 0;
		commandCnt = 0;
		can = 0;
	} else {
		can = new MotorController(portName);
		CommPort = 0;
	}
	// Set hardware-specific calibration
	setJointsToMotorsRatio(DEFAULT_X_GEAR_RATIO * DEFAULT_X_ENCODER_RES / (2 * M_PI),
			DEFAULT_Y_GEAR_RATIO * DEFAULT_Y_ENCODER_RES / (2 * M_PI));
	setMotorsOffset(DEFAULT_X_MOTOR_OFFSET, DEFAULT_Y_MOTOR_OFFSET);
}

PTProxy::~PTProxy() {
}

bool PTProxy::isConnected() const {
	return (CommPort->isConnected());
}

void PTProxy::nextStep(void){
	if(CommPort != 0) {
		// Always read position
		commandArray[commandCnt++] = NF_COMMAND_ReadDrivesPosition;
		// Always read status
		commandArray[commandCnt++] = NF_COMMAND_ReadDrivesStatus;

		// Try to receive a reply to the previous command
		while(CommPort->read(&(rxBuf[rxCnt]), 1) > 0){
			NF_Interpreter(&NFComBuf, (uint8_t*)rxBuf, (uint8_t*)&rxCnt, rxCommandArray, &rxCommandCnt);
		}

		// If a command send requested
		if(commandCnt > 0) {
			txCnt = NF_MakeCommandFrame(&NFComBuf, txBuf, (const uint8_t*)commandArray, commandCnt, NF_AddressBase);
			// Clear communication request
			commandCnt = 0;
			// Send command frame to motor controllers
			CommPort->write(txBuf, txCnt);
		}
	}
	
	if(can != 0) {
		uint32_t status1, status2;
		uint8_t mode1, mode2;
		can->getStatus2(0, status1, mode1, status2, mode2);
		can->getPosition2(0, NFComBuf.ReadDrivesPosition.data[0], NFComBuf.ReadDrivesPosition.data[1]);
		
		NFComBuf.ReadDrivesStatus.data[0] = status1 & 0xFFFF;
		NFComBuf.ReadDrivesStatus.data[1] = status2 & 0xFFFF;
		
		NFComBuf.SetDrivesMode.data[0] = mode1;
		NFComBuf.SetDrivesMode.data[1] = mode2;
	}
}

void PTProxy::startSynchronization(void){
	if(CommPort != 0){
		NFComBuf.SetDrivesMode.data[0] = NF_DrivesMode_SYNC_POS0;
		NFComBuf.SetDrivesMode.data[1] = NF_DrivesMode_SYNC_POS0;
		commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
	}
	
	if(can != 0) {
		can->setMode(0, NF_DrivesMode_SYNC_POS0);
		can->setMode(1, NF_DrivesMode_SYNC_POS0);
	}
}

bool PTProxy::isSynchronized(){
	if((NFComBuf.ReadDrivesStatus.data[0] & NF_DrivesStatus_Synchronized)
			&& (NFComBuf.ReadDrivesStatus.data[1] & NF_DrivesStatus_Synchronized))
		return true;
	return false;
}

int PTProxy::setMotorSpeed(float sx, float sy){
	if(!isSynchronized())
		return 1;
	if(CommPort != 0) {
		NFComBuf.SetDrivesSpeed.data[0] = sx;
		NFComBuf.SetDrivesSpeed.data[1] = sy;
		commandArray[commandCnt++] = NF_COMMAND_SetDrivesSpeed;

		if((NFComBuf.SetDrivesMode.data[0] != NF_DrivesMode_SPEED)
				|| (NFComBuf.SetDrivesMode.data[1] != NF_DrivesMode_SPEED)){
			NFComBuf.SetDrivesMode.data[0] = NF_DrivesMode_SPEED;
			NFComBuf.SetDrivesMode.data[1] = NF_DrivesMode_SPEED;
			commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
		}
	}
	
	if(can != 0) {
		if((NFComBuf.SetDrivesMode.data[0] != NF_DrivesMode_SPEED)
				|| (NFComBuf.SetDrivesMode.data[1] != NF_DrivesMode_SPEED)){
			can->setMode(0, NF_DrivesMode_SPEED);
			can->setMode(1, NF_DrivesMode_SPEED);
		}
		can->setVelocity2(0, (int32_t)sx, (int32_t)sy);
	}
	return 0;
}

int PTProxy::setMotorPosition(float px, float py){
	if(!isSynchronized())
		return 1;
	if(CommPort != 0) {
		NFComBuf.SetDrivesPosition.data[0] = px;
		NFComBuf.SetDrivesPosition.data[1] = py;
		commandArray[commandCnt++] = NF_COMMAND_SetDrivesPosition;
		
		if((NFComBuf.SetDrivesMode.data[0] != NF_DrivesMode_POSITION)
				|| (NFComBuf.SetDrivesMode.data[1] != NF_DrivesMode_POSITION)){
			NFComBuf.SetDrivesMode.data[0] = NF_DrivesMode_POSITION;
			NFComBuf.SetDrivesMode.data[1] = NF_DrivesMode_POSITION;
			commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
		}
	}
	
	if(can != 0) {
		if((NFComBuf.SetDrivesMode.data[0] != NF_DrivesMode_POSITION)
				|| (NFComBuf.SetDrivesMode.data[1] != NF_DrivesMode_POSITION)){
			can->setMode(0, NF_DrivesMode_POSITION);
			can->setMode(1, NF_DrivesMode_POSITION);
		}
		
		can->setPosition2(0, (int32_t)px, (int32_t)py);
	}
	return 0;
}

int PTProxy::setMotorPositionWithSpeed(float px, float py, float sx, float sy){
	if(!isSynchronized())
		return 1;
	if(CommPort != 0) {
		NFComBuf.SetDrivesSpeed.data[0] = sx;
		NFComBuf.SetDrivesSpeed.data[1] = sy;
		commandArray[commandCnt++] = NF_COMMAND_SetDrivesSpeed;
		NFComBuf.SetDrivesPosition.data[0] = px;
		NFComBuf.SetDrivesPosition.data[1] = py;
		commandArray[commandCnt++] = NF_COMMAND_SetDrivesPosition;

		if((NFComBuf.SetDrivesMode.data[0] != NF_DrivesMode_POSITION)
				|| (NFComBuf.SetDrivesMode.data[1] != NF_DrivesMode_POSITION)){
			NFComBuf.SetDrivesMode.data[0] = NF_DrivesMode_POSITION;
			NFComBuf.SetDrivesMode.data[1] = NF_DrivesMode_POSITION;
			commandArray[commandCnt++] = NF_COMMAND_SetDrivesMode;
		}
	}
	
	if(can != 0) {
		if((NFComBuf.SetDrivesMode.data[0] != NF_DrivesMode_POSITION)
				|| (NFComBuf.SetDrivesMode.data[1] != NF_DrivesMode_POSITION)){
			can->setMode(0, NF_DrivesMode_POSITION);
			can->setMode(1, NF_DrivesMode_POSITION);
		}
		
		can->setVelocity2(0, (int32_t)sx, (int32_t)sy);
		can->setPosition2(0, (int32_t)px, (int32_t)py);
	}
	return 0;
}

int PTProxy::setJointSpeed(float sx, float sy) {
	return setMotorSpeed(sx * xJointsToMotorsRatio, sy * yJointsToMotorsRatio);
}

int PTProxy::setJointPosition(float px, float py) {
	return setMotorPosition(px * xJointsToMotorsRatio + xMotorsOffset, py * yJointsToMotorsRatio + yMotorsOffset);
}

int PTProxy::setJointPositionWithSpeed(float px, float py, float sx, float sy) {
	return setMotorPositionWithSpeed(px * xJointsToMotorsRatio + xMotorsOffset,
									 py * yJointsToMotorsRatio + yMotorsOffset,
									 sx * xJointsToMotorsRatio,
									 sy * yJointsToMotorsRatio);
}

void PTProxy::getMotorSpeed(float& sx, float& sy) {
	sx = NFComBuf.ReadDrivesSpeed.data[0];
	sy = NFComBuf.ReadDrivesSpeed.data[1];
}

void PTProxy::getMotorPosition(float& px, float& py) {
	px = NFComBuf.ReadDrivesPosition.data[0] - xMotorsOffset;
	py = NFComBuf.ReadDrivesPosition.data[1] - yMotorsOffset;
}

void PTProxy::getJointSpeed(float& sx, float& sy) {
	sx = NFComBuf.ReadDrivesSpeed.data[0] / xJointsToMotorsRatio;
	sy = NFComBuf.ReadDrivesSpeed.data[1] / yJointsToMotorsRatio;
}

void PTProxy::getJointPosition(float& px, float& py) {
	px = (NFComBuf.ReadDrivesPosition.data[0] - xMotorsOffset) / xJointsToMotorsRatio;
	py = (NFComBuf.ReadDrivesPosition.data[1] - yMotorsOffset) / yJointsToMotorsRatio;
}

void PTProxy::setJointsToMotorsRatio(float jtmx, float jtmy){
	xJointsToMotorsRatio = jtmx;
	yJointsToMotorsRatio = jtmy;
}

void PTProxy::getJointsToMotorsRatio(float & jtmx, float & jtmy){
	jtmx = xJointsToMotorsRatio;
	jtmy = yJointsToMotorsRatio;
}

void PTProxy::setMotorsOffset(int offx, int offy){
	xMotorsOffset = offx;
	yMotorsOffset = offy;
}

void PTProxy::getMotorsOffset(int & offx, int & offy){
	offx = xMotorsOffset;
	offy = yMotorsOffset;
}
