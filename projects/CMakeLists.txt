target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/Irq_handlers.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/System_startup.cpp
)
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/apps
)

add_subdirectory(thread)
