cmake_minimum_required(VERSION 3.8)
project(openarm_skeleton_v1_2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    description_contract
    test/test_description.py
    TIMEOUT 60
  )
endif()

ament_package()
