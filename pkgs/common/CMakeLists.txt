cmake_minimum_required(VERSION 3.8)
project(common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# let the compiler search for headers in the include folder
include_directories(include)
 
# define a library target called my_value_converter_library
add_library(common src/common.cpp)
 
# this line to exports the library
ament_export_targets(common HAS_LIBRARY_TARGET)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/common
  DESTINATION include
)
 
install(
  TARGETS common
  EXPORT common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

ament_package()
