cmake_minimum_required(VERSION 3.8)
project(openarm_skeleton_v1_2_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY behavior_trees config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS scripts/check_nav2_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    navigation_contract
    test/test_navigation_contract.py
    TIMEOUT 60
  )
endif()

ament_package()
