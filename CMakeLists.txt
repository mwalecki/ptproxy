project(PtProxy)
cmake_minimum_required(VERSION 2.6)

set(PTPROXY_SRCS
    PTProxy.cpp
    nfv2_config.c
)

set(NF_SRCS
    nf/nfv2.c
)

set(SERIALCOMM_SRCS
    serialcomm/serialcomm.cpp
)

set(SRCS ${PTPROXY_SRCS} ${NF_SRCS} ${SERIALCOMM_SRCS})

add_library(ptproxy STATIC ${SRCS})