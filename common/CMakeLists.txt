# Source/Include files
include_directories(".")
include_directories("include/" )                 # common includes
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("../lcm-types/cpp/")
include_directories("../third-party/inih")       # inih includes
include_directories("../third-party/osqp/include")       # osqp includes
include_directories("../third-party/ParamHandler")
include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
file(GLOB_RECURSE sources "src/*.cpp")              # common library cpp files

# Library
add_library(biomimetics SHARED ${sources} include/Controllers/TouchdownDetect.h src/Controllers/TouchdownDetect.cpp)       # produce a library used by sim/robot
target_link_libraries(biomimetics inih dynacore_param_handler JCQP osqp)

if(CMAKE_SYSTEM_NAME MATCHES Linux)
# Pull in Google Test
include(CTest)
if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()

endif(CMAKE_SYSTEM_NAME MATCHES Linux)

