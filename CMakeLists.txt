cmake_minimum_required(VERSION 2.8)
project(csc232)
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR} CACHE PATH "default install prefix" FORCE)
set(ROS_PATH_PREFIX /opt/ros/indigo)

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
find_package( OpenGL REQUIRED )
find_package( GLUT REQUIRED )

#find program for (navigator, simulator,guiprocess,localization)
find_program(GENGETOPT gengetopt)
message("${GENGETOPT}")
execute_process(COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/ros_navigator.ggo --file-name ros_navigator_cmdline --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts)
execute_process(COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/ros_simulator.ggo --file-name ros_simulator_cmdline --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts)
execute_process(COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/ros_localization.ggo --file-name ros_localization_cmdline --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts)
execute_process(COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/gui_process.ggo --file-name gui_process_cmdline --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts)
execute_process(COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/ros_executive.ggo --file-name ros_executive_cmdline --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts)



find_path(ROS_INCLUDE_DIR NAMES ros/ros.h HINTS CMAKE_INSTALL_PREFIX/include)
include_directories(${ROS_INCLUDE_DIR} ${PROJECT_BINARY_DIR})
file(GLOB ROS_LIBRARIES "${ROS_PATH_PREFIX}/lib/*.so*")
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} )

qt_wrap_cpp(gui MOC_SRCS gui.h)

#add_executable,target_link and install for (N,S,L,GUI)
add_executable(ros_navigator ros_navigator.cpp ${CMAKE_CURRENT_BINARY_DIR}/ros_navigator_cmdline.c)
target_link_libraries(ros_navigator ${ROS_LIBRARIES})
install(TARGETS ros_navigator DESTINATION bin)

add_executable(ros_simulator ros_simulator.cpp ${CMAKE_CURRENT_BINARY_DIR}/ros_simulator_cmdline.c)
target_link_libraries(ros_simulator ${ROS_LIBRARIES})
install(TARGETS ros_simulator DESTINATION bin)

add_executable(ros_localization ros_localization.cpp ${CMAKE_CURRENT_BINARY_DIR}/ros_localization_cmdline.c)
target_link_libraries(ros_localization ${ROS_LIBRARIES})
install(TARGETS ros_localization DESTINATION bin)

add_executable(ros_executive ros_executive.cpp ${CMAKE_CURRENT_BINARY_DIR}/ros_executive_cmdline.c)
target_link_libraries(ros_executive ${ROS_LIBRARIES})
install(TARGETS ros_executive DESTINATION bin)

add_executable(gui_process gui_process.cc gui.cc ${MOC_SRCS} 
${CMAKE_CURRENT_BINARY_DIR}/gui_process_cmdline.c)
target_link_libraries(gui_process ${ROS_LIBRARIES} ${QT_LIBRARIES} ${OPENGL_LIBRARIES} )
install( TARGETS gui_process DESTINATION bin )


