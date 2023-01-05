add_library(ecl_geo geo.cpp )

target_include_directories(ecl_geo PUBLIC ${ECL_SOURCE_DIR})

target_link_libraries(ecl_geo PRIVATE ecl)