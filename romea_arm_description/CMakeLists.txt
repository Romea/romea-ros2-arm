cmake_minimum_required(VERSION 3.5)
project(romea_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_arm_description.py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/controller_manager.yaml  test/controller_manager.yaml)
  ament_add_pytest_test(test_gps_description test/test_arm_description.py)
endif()

ament_package()
