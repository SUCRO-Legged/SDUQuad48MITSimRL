cmake_minimum_required(VERSION 3.5)
project(MITSimRL)
add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

option(REAL_ROBOT_BUILD "use compiler flags for real robot computer" OFF)
set(BUILD_TYPE_RELEASE TRUE)

option(NO_SIM "Do not build simulator" OFF)

if(REAL_ROBOT_BUILD)
  SET (THIS_COM "../" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
  set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wsign-promo")
  set(CMAKE_C_FLAGS "-O3  -ggdb -std=gnu99 -I.")
  message("**** Real Robot Build Enabled ****")
else(REAL_ROBOT_BUILD)
	SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
	CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
		${CMAKE_BINARY_DIR}/Configuration.h)

	if(CMAKE_SYSTEM_NAME MATCHES Linux)
		set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
		-Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
		-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
		-Wsign-promo")
	elseif(APPLE)
		set(CMAKE_CXX_FLAGS "-O3 -march=native -ggdb -Wall \
		-Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
		-Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
		-Wshadow -Wsign-promo")
		include_directories("/usr/local/include/")   # lcm includes
	endif()

	set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
	message("**** Real Robot Build Enabled ****")
endif(REAL_ROBOT_BUILD)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(robot)
add_subdirectory(third-party)
add_subdirectory(common)

if(NO_SIM)
else(NO_SIM)
	add_subdirectory(sim)
endif()

add_subdirectory(user)
