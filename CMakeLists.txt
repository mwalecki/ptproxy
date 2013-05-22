project(PtProxy)
cmake_minimum_required(VERSION 2.6)

set(PTPROXY_SRCS
    PTProxy.cpp
    nfv2_config.c
)

set(CAN_SRC
    can/CANDev.cpp
    can/MotorController.cpp
)

set(NF_SRCS
    nf/nfv2.c
)

set(SERIALCOMM_SRCS
    serialcomm/serialcomm.cpp
)

set(SRCS ${PTPROXY_SRCS} ${CAN_SRC} ${NF_SRCS} ${SERIALCOMM_SRCS})

add_library(ptproxy STATIC ${SRCS})
