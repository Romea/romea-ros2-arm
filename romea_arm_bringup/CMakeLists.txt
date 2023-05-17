cmake_minimum_required(VERSION 3.5)
project(romea_arm_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_arm_bringup.py)

install(PROGRAMS
  scripts/urdf_description.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_arm_bringup.yaml test_arm_bringup.yaml)
  ament_add_pytest_test(test_arm_bringup test/test_arm_bringup.py)
  ament_add_pytest_test(test_urdf_description test/test_urdf_description.py)
endif()

ament_package()
