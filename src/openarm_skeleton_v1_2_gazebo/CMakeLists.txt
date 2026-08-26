cmake_minimum_required(VERSION 3.8)
project(openarm_skeleton_v1_2_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS
    scripts/check_cmd_vel_motion.py
    scripts/cmd_vel_watchdog.py
    scripts/demo_indoor_route.py
    scripts/demo_skeleton_lift.py
    scripts/spawn_entity.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    gazebo_contract
    test/test_gazebo_contract.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    safety_helpers
    test/test_safety_helpers.py
    TIMEOUT 60
  )
endif()

ament_package()
