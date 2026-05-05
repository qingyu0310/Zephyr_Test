if(CONFIG_TPC_CHASSIS_TO_CAN)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/chassis_to_can)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/chassis_to_can/chassis_to_can.cpp)
endif()

if(CONFIG_TPC_REMOTE_TO)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remote_to)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/remote_to/remote_to.cpp)
endif()
