add_library(ecl_eskf eskf.cpp geskf.cpp utils.cpp imu_down_sampler.cpp eskf_runner.h)

target_include_directories(ecl_eskf PUBLIC ${ECL_SOURCE_DIR})

target_link_libraries(ecl_eskf PRIVATE ecl)