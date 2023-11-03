cmake_minimum_required(VERSION 3.8)
project(human_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()


  add_launch_test(
    test/test_description.launch.py
    TARGET "human_description"
    ARGS "height:=1.7"
  )

endif()

ament_auto_package(INSTALL_TO_SHARE config launch urdf)
