cmake_minimum_required(VERSION 3.5)
project(cpp_example_service)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#
find_package(geometry_msgs REQUIRED)
find_package(cpp_example_service REQUIRED)

rosidl_generate_interfaces(
        ${PROJECT_NAME}
        "srv/TurningControl.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#
file(GLOB CPP_FILES src/*.cpp)
foreach( FILE_PATH ${CPP_FILES} )
        get_filename_component(FILE_NAME ${FILE_PATH} NAME_WE)
        add_executable( ${FILE_NAME} ${FILE_PATH} )

        ament_target_dependencies( ${FILE_NAME}
                "rclcpp"
                "cpp_example_service"
                "geometry_msgs")

        install(TARGETS ${FILE_NAME}
                DESTINATION lib/${PROJECT_NAME})

endforeach(FILE_PATH ${APP_SOURCES})

ament_package()
