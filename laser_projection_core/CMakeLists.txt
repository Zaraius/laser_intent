cmake_minimum_required(VERSION 3.16)
project(laser_projection_core)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/projector.cpp
  src/udp_backend.cpp
  src/helios_backend.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_DL_LIBS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_library(fake_helios_dac_api SHARED test/fake_helios_dac_api.cpp)
  target_compile_features(fake_helios_dac_api PRIVATE cxx_std_17)
  ament_add_gtest(test_projector test/test_projector.cpp)
  target_compile_definitions(test_projector PRIVATE
    FAKE_HELIOS_LIBRARY="$<TARGET_FILE:fake_helios_dac_api>")
  target_link_libraries(test_projector ${PROJECT_NAME} fake_helios_dac_api
    ${CMAKE_DL_LIBS})
endif()

ament_package()
