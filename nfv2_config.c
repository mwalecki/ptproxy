#include "nf/nfv2.h"

void NFv2_Config2(NF_STRUCT_ComBuf *NFComBuf, uint8_t myAddress, uint8_t slaveAddress){

    NFComBuf->myAddress = myAddress;
    NFComBuf->ReadDeviceVitals.addr[0] = slaveAddress;
    NFComBuf->ReadDeviceVitals.addr[1] = slaveAddress;

    NFComBuf->SetDrivesMode.addr[0] = slaveAddress;
    NFComBuf->SetDrivesMode.addr[1] = slaveAddress;

    NFComBuf->SetDrivesPWM.addr[0] = slaveAddress;
    NFComBuf->SetDrivesPWM.addr[1] = slaveAddress;

    NFComBuf->SetDrivesSpeed.addr[0] = slaveAddress;
    NFComBuf->SetDrivesSpeed.addr[1] = slaveAddress;

    NFComBuf->SetDrivesPosition.addr[0] = slaveAddress;
    NFComBuf->SetDrivesPosition.addr[1] = slaveAddress;

    NFComBuf->ReadDrivesPosition.addr[0] = slaveAddress;
    NFComBuf->ReadDrivesPosition.addr[1] = slaveAddress;

    NFComBuf->ReadDrivesStatus.addr[0] = slaveAddress;
    NFComBuf->ReadDrivesStatus.addr[1] = slaveAddress;

    NFComBuf->ReadAnalogInputs.addr[0] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[1] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[2] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[3] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[4] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[5] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[6] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[7] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[8] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[9] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[10] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[11] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[12] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[13] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[14] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[15] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[16] = slaveAddress;
    NFComBuf->ReadAnalogInputs.addr[17] = slaveAddress;

    NFComBuf->SetDigitalOutputs.addr[0] = slaveAddress;
    NFComBuf->SetDigitalOutputs.addr[1] = slaveAddress;
}

