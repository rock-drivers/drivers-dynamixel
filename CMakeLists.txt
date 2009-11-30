PROJECT(dynamixel)
SET(PROJECT_VERSION 1.0)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
INCLUDE(FindPkgConfig)

INCLUDE_DIRECTORIES(.)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

pkg_check_modules(IO REQUIRED "iodrivers_base")
INCLUDE_DIRECTORIES(${IO_INCLUDE_DIRS})
LINK_DIRECTORIES(${IO_LIBRARY_DIRS})

ADD_LIBRARY(dynamixel SHARED dynamixel.cc dxlib/dxseries.c)
TARGET_LINK_LIBRARIES(dynamixel ${IO_LIBRARIES})

ADD_EXECUTABLE(dynamixel_test dynamixel_test.cc)
ADD_EXECUTABLE(dynamixel_scan dynamixel_scan.cc)
TARGET_LINK_LIBRARIES(dynamixel_test dynamixel)
TARGET_LINK_LIBRARIES(dynamixel_scan dynamixel)
INSTALL(TARGETS dynamixel  LIBRARY DESTINATION lib)
INSTALL(FILES dynamixel.hh DESTINATION include)

CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)

CONFIGURE_FILE(dynamixel.pc.in dynamixel.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/dynamixel.pc DESTINATION lib/pkgconfig)

