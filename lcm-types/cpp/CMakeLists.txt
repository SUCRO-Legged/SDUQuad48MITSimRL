cmake_minimum_required(VERSION 3.5)

add_executable(robot_record leg_record.cpp)
target_link_libraries(robot_record lcm)

