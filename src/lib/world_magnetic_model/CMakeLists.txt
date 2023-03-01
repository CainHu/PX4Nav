add_library(ecl_mag_world geo_mag_declination.cpp)

target_include_directories(ecl_mag_world PUBLIC ${ECL_SOURCE_DIR})

target_link_libraries(ecl_mag_world PRIVATE ecl)