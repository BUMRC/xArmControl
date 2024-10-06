cmake_minimum_required(VERSION 3.22)
project(xarm_description)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
hardware_interface
pluginlib
rclcpp
rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
xarm_description
SHARED
hardware/xarm_hardware.cpp
)
target_compile_features(xarm_description PUBLIC cxx_std_17)
target_include_directories(xarm_description PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/xarm_description>
)
ament_target_dependencies(
xarm_description PUBLIC
${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(xarm_description PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "XARM_DESCRIPTION_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface xarm_description.xml)

# INSTALL
install(
DIRECTORY hardware/include/
DESTINATION include/xarm_description
)
install(
DIRECTORY description/launch description/ros2_control description/urdf description/rviz
DESTINATION share/xarm_description
)
install(
DIRECTORY bringup/launch bringup/config
DESTINATION share/xarm_description
)
install(TARGETS xarm_description
EXPORT export_xarm_description
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_xarm_description HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()