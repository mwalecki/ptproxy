
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <inttypes.h>

#include <iostream>
#include <cstring>

#ifndef __XENO__
	#define rt_dev_socket socket
	#define rt_dev_setsockopt setsockopt
	#define rt_dev_bind bind
	#define rt_dev_recvfrom recvfrom
	#define rt_dev_sendto sendto
	#define rt_dev_close close
#else
	#include <rtdm/rtcan.h>
#endif

#include "CANDev.h"

CANDev::CANDev(std::string dev_name) {
  struct sockaddr_can addr;
  struct ifreq ifr;
  
  if((dev = rt_dev_socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    std::cout<< "Error while opening socket" << std::endl;
    dev = -1;
  }
  
  strcpy(ifr.ifr_name, dev_name.c_str());
  ioctl(dev, SIOCGIFINDEX, &ifr);
  
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex; 
  
  if(rt_dev_bind(dev, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    std::cout << "Error in socket bind" << std::endl;
    rt_dev_close(dev);
    dev = -1;
  }
}

CANDev::~CANDev() {
  if(dev > -1) {
	  rt_dev_close(dev);
  }
}

void CANDev::send(const uint32_t can_id, const uint8_t len, const uint8_t *data){
  struct can_frame frame;
  
  frame.can_id = can_id;
  frame.can_dlc = len;
  
  memcpy(frame.data, data, len);
  write(dev, &frame, sizeof(frame));
}

uint32_t CANDev::waitForReply(uint32_t can_id, uint8_t *data) {
  struct can_frame frame;
  
  // search frame buffer
  for(size_t i = 0; i < frame_buf.size(); i++) {

    if(frame_buf[i].can_id == can_id) {
      memcpy(data, frame_buf[i].data, frame_buf[i].can_dlc);
      frame_buf.erase(frame_buf.begin()+i);
      return can_id;
    }
  }
  
  // wait for new data
  while(1) {
    size_t ret = read(dev, &frame, sizeof(frame));
    if(ret != sizeof(frame)) {
      continue;
    }
    

    if(frame.can_id == can_id) {
     memcpy(data, frame.data, frame.can_dlc);
      return can_id;
    }
    
    frame_buf.push_back(frame);
  }
}


uint32_t CANDev::AddFilter( const CANDev::Filter& filter ) {

  //std::cout << "RTSocketCAN::AddFilter" << std::endl;

  if( n_filters_ < CANDev::MAX_NUM_FILTERS ){

    /*
    // Avoid duplicates
    for( size_t i=0; i<n_filters_; i++ ){
    if( filters[i].can_mask == filter.mask && filters[i].can_id == filter.id )
    { return CANBus::ESUCCESS; }
    }
    */

    filters_[n_filters_].can_mask = filter.mask_;
    filters_[n_filters_].can_id   = filter.id_;
    n_filters_++;

    // Set the filter to the socket
    int rt_error = 0;
    if( rt_error = rt_dev_setsockopt(
    	  dev,
          SOL_CAN_RAW,
          CAN_RAW_FILTER,
          filters_,
          n_filters_*sizeof(struct can_filter) ) )
    {
      std::cerr << "Couldn't set the socket filter ("<<filter.mask_<<", "<<filter.id_<<") on socket descriptor "<<dev<<": " <<strerror(-rt_error)<< std::endl;
      return 1;
    }

    return 0;
  } else {
    std::cerr << "Reached maximum number of filters." << std::endl;
    return 1;
  }

  return 1;

}

