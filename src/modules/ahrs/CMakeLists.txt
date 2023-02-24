add_library(ecl_ahrs ahrs.cpp)

target_include_directories(ecl_ahrs PUBLIC ${ECL_SOURCE_DIR})

target_link_libraries(ecl_ahrs PRIVATE ecl)