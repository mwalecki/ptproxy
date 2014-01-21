#ifndef _CAN_DEV_H_
#define _CAN_DEV_H_

#include <string>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>

class CANDev {
public:
  CANDev(std::string dev);
  ~CANDev();
  
  void send(uint32_t can_id, uint8_t len, const uint8_t *data);
  uint32_t waitForReply(uint32_t can_id, uint8_t *data);

  typedef unsigned short id_t; // FIXME: formerly ID
  typedef unsigned short mask_t; // FIXME: formerly Mask

  // CAN Filter
  /**
    CAN filters are used by the kernel to filter frames that are not aimed for
    a specific node id.
    */
  class Filter {
  public:
	  CANDev::mask_t mask_;
	  CANDev::id_t id_;

    Filter( CANDev::mask_t mask, CANDev::id_t id ) :
      mask_( mask ),
      id_( id )
    { }
  };

    //! Add a filter to the RTSocketCAN device
    uint32_t AddFilter( const CANDev::Filter& filter );
protected:
private:
  int dev;
  std::vector<can_frame> frame_buf;

  //! CAN filters
  /**
     The default maximum number of CAN filter in Xenomai is 16. Howerver, this
     limit can be increased when configuring the kernel.
   */
  static const size_t MAX_NUM_FILTERS = 32;
  can_filter filters_[MAX_NUM_FILTERS];
  size_t n_filters_;
};

#endif

