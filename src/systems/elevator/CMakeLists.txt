gz_add_system(elevator
  SOURCES
    Elevator.cc
    utils/DoorTimer.cc
    utils/JointMonitor.cc
  PUBLIC_LINK_LIBS
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
  PRIVATE_INCLUDE_DIRS
    vender/afsm/include
    vender/metapushkin/include
)
