cmake_minimum_required(VERSION 3.5)
project(racecar_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install URDF/Xacro files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install meshes
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install Gazebo model files
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Declare the ament package
ament_package()

# Include directories for the package
include_directories(
)
