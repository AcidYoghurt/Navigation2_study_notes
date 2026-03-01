cmake_minimum_required(VERSION 3.8)
project(nav2_custom_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 导出库
ament_auto_add_library(
        nav2_custom_planner           # 【参数1】库的名字（CMake target 名）
        SHARED                        # 【参数2】库的类型
        src/nav2_custom_planner.cpp   # 【参数3】库的源文件列表
)

# 导出插件描述文件
pluginlib_export_plugin_description_file(nav2_custom_planner plugin.xml)

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

ament_auto_package()
