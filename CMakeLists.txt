cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 14)

set(PROJECT_NAME JointCalibController)
set(PROJECT_DESCRIPTION "mc-rtc controller for fixed trajectory joint motion.")
set(PROJECT_URL "https://github.com/rohanpsingh/mc_joint_calib_controller")

project(${PROJECT_NAME} CXX)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
find_package(WalkerPolicy REQUIRED)

set(MC_RTC_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")
mc_rtc_set_prefix(CONTROLLER mc_controller)
mc_rtc_set_prefix(STATES mc_controller/${PROJECT_NAME}/states)

add_subdirectory(src)

set(AROBASE "@")

configure_file(etc/JointCalibController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/JointCalibController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/JointCalibController.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
