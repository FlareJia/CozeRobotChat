# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "file_transfer: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(file_transfer_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv" NAME_WE)
add_custom_target(_file_transfer_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "file_transfer" "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(file_transfer
  "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/file_transfer
)

### Generating Module File
_generate_module_cpp(file_transfer
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/file_transfer
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(file_transfer_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(file_transfer_generate_messages file_transfer_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv" NAME_WE)
add_dependencies(file_transfer_generate_messages_cpp _file_transfer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(file_transfer_gencpp)
add_dependencies(file_transfer_gencpp file_transfer_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS file_transfer_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(file_transfer
  "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/file_transfer
)

### Generating Module File
_generate_module_eus(file_transfer
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/file_transfer
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(file_transfer_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(file_transfer_generate_messages file_transfer_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv" NAME_WE)
add_dependencies(file_transfer_generate_messages_eus _file_transfer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(file_transfer_geneus)
add_dependencies(file_transfer_geneus file_transfer_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS file_transfer_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(file_transfer
  "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/file_transfer
)

### Generating Module File
_generate_module_lisp(file_transfer
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/file_transfer
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(file_transfer_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(file_transfer_generate_messages file_transfer_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv" NAME_WE)
add_dependencies(file_transfer_generate_messages_lisp _file_transfer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(file_transfer_genlisp)
add_dependencies(file_transfer_genlisp file_transfer_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS file_transfer_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(file_transfer
  "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/file_transfer
)

### Generating Module File
_generate_module_nodejs(file_transfer
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/file_transfer
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(file_transfer_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(file_transfer_generate_messages file_transfer_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv" NAME_WE)
add_dependencies(file_transfer_generate_messages_nodejs _file_transfer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(file_transfer_gennodejs)
add_dependencies(file_transfer_gennodejs file_transfer_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS file_transfer_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(file_transfer
  "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/file_transfer
)

### Generating Module File
_generate_module_py(file_transfer
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/file_transfer
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(file_transfer_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(file_transfer_generate_messages file_transfer_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/szhr/CozeRobotChat/ros_ws/src/file_transfer/srv/FileTransfer.srv" NAME_WE)
add_dependencies(file_transfer_generate_messages_py _file_transfer_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(file_transfer_genpy)
add_dependencies(file_transfer_genpy file_transfer_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS file_transfer_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/file_transfer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/file_transfer
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(file_transfer_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/file_transfer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/file_transfer
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(file_transfer_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/file_transfer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/file_transfer
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(file_transfer_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/file_transfer)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/file_transfer
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(file_transfer_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/file_transfer)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/file_transfer\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/file_transfer
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(file_transfer_generate_messages_py std_msgs_generate_messages_py)
endif()
