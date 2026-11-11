cmake_minimum_required(VERSION 3.14)
project(sonar_context VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(sonar_context src/sonar_context.cpp)
add_library(sonar_context::sonar_context ALIAS sonar_context)
target_include_directories(sonar_context PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(sonar_context PUBLIC Eigen3::Eigen)
set_target_properties(sonar_context PROPERTIES POSITION_INDEPENDENT_CODE ON)

option(SONAR_CONTEXT_BUILD_TOOLS "Build the command-line tools and tests" ON)
if(SONAR_CONTEXT_BUILD_TOOLS AND CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  add_executable(sonar_context_distances tools/distances.cpp)
  target_link_libraries(sonar_context_distances sonar_context)
  enable_testing()
  add_executable(test_sonar_context tests/test_sonar_context.cpp)
  target_link_libraries(test_sonar_context sonar_context)
  add_test(NAME sonar_context COMMAND test_sonar_context)
endif()

include(GNUInstallDirs)
install(TARGETS sonar_context EXPORT sonar_contextTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT sonar_contextTargets NAMESPACE sonar_context:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sonar_context)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/sonar_contextConfig.cmake
  "include(CMakeFindDependencyMacro)\nfind_dependency(Eigen3 3.3)\ninclude(\${CMAKE_CURRENT_LIST_DIR}/sonar_contextTargets.cmake)\n")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sonar_contextConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/sonar_context)
