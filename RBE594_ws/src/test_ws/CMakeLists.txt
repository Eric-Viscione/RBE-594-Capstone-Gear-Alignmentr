cmake_minimum_required(VERSION 3.22)
project(test_ws)

find_package(ament_cmake REQUIRED)

# Install assets so Gazebo & RViz can find them under:
# install/test_ws/share/test_ws/...
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/urdf")
  install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

# Optional: the MoveIt Setup Assistant marker file
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant")
  install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
