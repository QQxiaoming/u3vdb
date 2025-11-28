cmake_minimum_required(VERSION 3.10)
project(u3vdb LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libusb_win)
    add_subdirectory(libusb_win)

	add_executable(u3vdb main.cpp)
	target_include_directories(u3vdb PRIVATE ${LIBUSB_INCLUDE_DIR})
	target_link_libraries(u3vdb PRIVATE ${LIBUSB_LIBRARY} libusb)
else()
	find_package(PkgConfig REQUIRED)
	pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

	find_package(Threads REQUIRED)

	add_executable(u3vdb main.cpp)
	target_link_libraries(u3vdb PRIVATE PkgConfig::LIBUSB Threads::Threads)
endif()

install(TARGETS u3vdb RUNTIME DESTINATION bin)
