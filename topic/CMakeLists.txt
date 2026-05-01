if(CONFIG_TPC_MOTOR_CMD)
    target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/motor_cmd.cpp)
    target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endif()
