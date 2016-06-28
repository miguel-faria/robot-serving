# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robot_serving: 3 messages, 4 services")

set(MSG_I_FLAGS "-Irobot_serving:/home/miguel/catkin_ws/src/robot_serving/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robot_serving_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg" NAME_WE)
add_custom_target(_robot_serving_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_serving" "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg" "robot_serving/PMPPoint"
)

get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/Movement.srv" NAME_WE)
add_custom_target(_robot_serving_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_serving" "/home/miguel/catkin_ws/src/robot_serving/srv/Movement.srv" "robot_serving/PMPTraj:robot_serving/PMPPoint"
)

get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementFeedback.srv" NAME_WE)
add_custom_target(_robot_serving_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_serving" "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementFeedback.srv" ""
)

get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementSendTrajectory.srv" NAME_WE)
add_custom_target(_robot_serving_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_serving" "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementSendTrajectory.srv" "robot_serving/PMPTraj:robot_serving/PMPPoint"
)

get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementCancelTrajectory.srv" NAME_WE)
add_custom_target(_robot_serving_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_serving" "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementCancelTrajectory.srv" "robot_serving/PMPTraj:robot_serving/PMPPoint"
)

get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/Cups.msg" NAME_WE)
add_custom_target(_robot_serving_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_serving" "/home/miguel/catkin_ws/src/robot_serving/msg/Cups.msg" ""
)

get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg" NAME_WE)
add_custom_target(_robot_serving_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_serving" "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_serving
)
_generate_msg_cpp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/msg/Cups.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_serving
)
_generate_msg_cpp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_serving
)

### Generating Services
_generate_srv_cpp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementSendTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg;/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_serving
)
_generate_srv_cpp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementFeedback.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_serving
)
_generate_srv_cpp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/Movement.srv"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg;/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_serving
)
_generate_srv_cpp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementCancelTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg;/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_serving
)

### Generating Module File
_generate_module_cpp(robot_serving
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_serving
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robot_serving_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robot_serving_generate_messages robot_serving_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg" NAME_WE)
add_dependencies(robot_serving_generate_messages_cpp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/Movement.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_cpp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementFeedback.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_cpp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementSendTrajectory.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_cpp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementCancelTrajectory.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_cpp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/Cups.msg" NAME_WE)
add_dependencies(robot_serving_generate_messages_cpp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg" NAME_WE)
add_dependencies(robot_serving_generate_messages_cpp _robot_serving_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_serving_gencpp)
add_dependencies(robot_serving_gencpp robot_serving_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_serving_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_serving
)
_generate_msg_lisp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/msg/Cups.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_serving
)
_generate_msg_lisp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_serving
)

### Generating Services
_generate_srv_lisp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementSendTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg;/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_serving
)
_generate_srv_lisp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementFeedback.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_serving
)
_generate_srv_lisp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/Movement.srv"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg;/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_serving
)
_generate_srv_lisp(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementCancelTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg;/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_serving
)

### Generating Module File
_generate_module_lisp(robot_serving
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_serving
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robot_serving_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robot_serving_generate_messages robot_serving_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg" NAME_WE)
add_dependencies(robot_serving_generate_messages_lisp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/Movement.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_lisp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementFeedback.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_lisp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementSendTrajectory.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_lisp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementCancelTrajectory.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_lisp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/Cups.msg" NAME_WE)
add_dependencies(robot_serving_generate_messages_lisp _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg" NAME_WE)
add_dependencies(robot_serving_generate_messages_lisp _robot_serving_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_serving_genlisp)
add_dependencies(robot_serving_genlisp robot_serving_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_serving_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving
)
_generate_msg_py(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/msg/Cups.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving
)
_generate_msg_py(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving
)

### Generating Services
_generate_srv_py(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementSendTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg;/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving
)
_generate_srv_py(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementFeedback.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving
)
_generate_srv_py(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/Movement.srv"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg;/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving
)
_generate_srv_py(robot_serving
  "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementCancelTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg;/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving
)

### Generating Module File
_generate_module_py(robot_serving
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robot_serving_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robot_serving_generate_messages robot_serving_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/PMPTraj.msg" NAME_WE)
add_dependencies(robot_serving_generate_messages_py _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/Movement.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_py _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementFeedback.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_py _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementSendTrajectory.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_py _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/srv/RobotMovementCancelTrajectory.srv" NAME_WE)
add_dependencies(robot_serving_generate_messages_py _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/Cups.msg" NAME_WE)
add_dependencies(robot_serving_generate_messages_py _robot_serving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/miguel/catkin_ws/src/robot_serving/msg/PMPPoint.msg" NAME_WE)
add_dependencies(robot_serving_generate_messages_py _robot_serving_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_serving_genpy)
add_dependencies(robot_serving_genpy robot_serving_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_serving_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_serving)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_serving
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(robot_serving_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(robot_serving_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_serving)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_serving
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(robot_serving_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(robot_serving_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_serving
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(robot_serving_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(robot_serving_generate_messages_py geometry_msgs_generate_messages_py)
