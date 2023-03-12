add_library(ecl_inav inav.cpp imu.cpp magnet.cpp gps.cpp ex_vision.cpp baro.cpp range.cpp opt_flow.cpp)

target_include_directories(ecl_inav PUBLIC ${ECL_SOURCE_DIR})

target_link_libraries(ecl_inav PRIVATE ecl)