#include "nfv2.h"

void NFv2_Config2(NF_STRUCT_ComBuf *NFComBuf, uint8_t myAddress, uint8_t deviceAddress){

    NFComBuf->myAddress = myAddress;
    NFComBuf->ReadDeviceVitals.addr[0] = deviceAddress;
    NFComBuf->ReadDeviceVitals.addr[1] = deviceAddress;

    NFComBuf->SetDrivesMode.addr[0] = deviceAddress;
    NFComBuf->SetDrivesMode.addr[1] = deviceAddress;

    NFComBuf->SetDrivesPWM.addr[0] = deviceAddress;
    NFComBuf->SetDrivesPWM.addr[1] = deviceAddress;

    NFComBuf->SetDrivesSpeed.addr[0] = deviceAddress;
    NFComBuf->SetDrivesSpeed.addr[1] = deviceAddress;

    NFComBuf->SetDrivesPosition.addr[0] = deviceAddress;
    NFComBuf->SetDrivesPosition.addr[1] = deviceAddress;

    NFComBuf->ReadDrivesPosition.addr[0] = deviceAddress;
    NFComBuf->ReadDrivesPosition.addr[1] = deviceAddress;

    NFComBuf->ReadDrivesStatus.addr[0] = deviceAddress;
    NFComBuf->ReadDrivesStatus.addr[1] = deviceAddress;

    NFComBuf->ReadAnalogInputs.addr[0] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[1] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[2] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[3] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[4] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[5] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[6] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[7] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[8] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[9] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[10] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[11] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[12] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[13] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[14] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[15] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[16] = deviceAddress;
    NFComBuf->ReadAnalogInputs.addr[17] = deviceAddress;

    NFComBuf->SetDigitalOutputs.addr[0] = deviceAddress;
    NFComBuf->SetDigitalOutputs.addr[1] = deviceAddress;
}

