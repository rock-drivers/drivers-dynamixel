rock_library(dynamixel
    SOURCES dynamixel.cpp dxseries.c dynamixel_iodriver.cpp
    HEADERS dxseries.h dynamixel.h dynamixel_iodriver.h dynamixel_types.hpp
    DEPS_PKGCONFIG iodrivers_base 
)

rock_executable(dynamixel_control_bin 
    SOURCES main_test.cpp
    DEPS dynamixel)

rock_executable(dynamixel_test_bin 
    SOURCES dynamixel_test.cpp
    DEPS dynamixel)
