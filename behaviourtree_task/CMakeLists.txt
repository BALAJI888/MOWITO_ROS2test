cmake_minimum_required(VERSION 3.5)
project(behavior_tree_task)

set(CMAKE_CXX_STANDARD 14)

# Find BehaviorTree.CPP
find_package(behaviortree_cpp REQUIRED)

# Add executable
add_executable(bt_task 
    src/main.cpp 
    src/nodes.cpp
    src/nodes.h
)

# Link against BehaviorTree.CPP
target_link_libraries(bt_task behaviortree_cpp)