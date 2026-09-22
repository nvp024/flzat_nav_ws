cmake_minimum_required(VERSION 3.8)
project(openarm_skeleton_v1_2_isaac)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(
  PROGRAMS
    scripts/check_isaac_runtime.py
    scripts/condition_isaac_cmd_vel.py
    scripts/correct_isaac_odometry.py
    scripts/openarm_isaac_sim.py
    scripts/prepare_isaac_urdf.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES scripts/isaac_scenes.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    isaac_contract
    test/test_isaac_contract.py
    TIMEOUT 60
  )
endif()

ament_package()
