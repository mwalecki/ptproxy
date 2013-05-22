#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <inttypes.h>
#include "CANDev.h"

class MotorController {
public:
  MotorController(std::string dev_name = "can0");
  ~MotorController();
  void setMode(const int dev_id,const uint8_t mode);
  
  void setVelocity(const int dev_id, int32_t vel);
  void setVelocity2(const int group_id, int32_t vel1, int32_t vel2);
  
  void setPosition(const int dev_id, int32_t pos);
  void setPosition2(const int group_id, int32_t pos1, int32_t pos2);
  
  void getStatus(const int dev_id, uint32_t &status, uint8_t &mode);
  void getStatus2(const int group_id, 
                  uint32_t &status1, uint8_t &mode1,
                  uint32_t &status2, uint8_t &mode2);
  void getStatus4(const int group_id, 
                  uint32_t &status1, uint8_t &mode1,
                  uint32_t &status2, uint8_t &mode2,
                  uint32_t &status3, uint8_t &mode3,
                  uint32_t &status4, uint8_t &mode4);
  
  void getPosition(const int dev_id, int32_t &pos);
  void getPosition2(const int group_id, int32_t &pos1, int32_t &pos2);
  void getPosition4(const int group_id,
                    int32_t &pos1, int32_t &pos2,
                    int32_t &pos3, int32_t &pos4);
  
protected:
private:
  CANDev dev;
};

#endif

