# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "music_client: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(music_client_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv" NAME_WE)
add_custom_target(_music_client_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "music_client" "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(music_client
  "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/music_client
)

### Generating Module File
_generate_module_cpp(music_client
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/music_client
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(music_client_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(music_client_generate_messages music_client_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv" NAME_WE)
add_dependencies(music_client_generate_messages_cpp _music_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(music_client_gencpp)
add_dependencies(music_client_gencpp music_client_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS music_client_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(music_client
  "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/music_client
)

### Generating Module File
_generate_module_eus(music_client
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/music_client
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(music_client_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(music_client_generate_messages music_client_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv" NAME_WE)
add_dependencies(music_client_generate_messages_eus _music_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(music_client_geneus)
add_dependencies(music_client_geneus music_client_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS music_client_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(music_client
  "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/music_client
)

### Generating Module File
_generate_module_lisp(music_client
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/music_client
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(music_client_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(music_client_generate_messages music_client_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv" NAME_WE)
add_dependencies(music_client_generate_messages_lisp _music_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(music_client_genlisp)
add_dependencies(music_client_genlisp music_client_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS music_client_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(music_client
  "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/music_client
)

### Generating Module File
_generate_module_nodejs(music_client
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/music_client
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(music_client_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(music_client_generate_messages music_client_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv" NAME_WE)
add_dependencies(music_client_generate_messages_nodejs _music_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(music_client_gennodejs)
add_dependencies(music_client_gennodejs music_client_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS music_client_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(music_client
  "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/music_client
)

### Generating Module File
_generate_module_py(music_client
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/music_client
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(music_client_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(music_client_generate_messages music_client_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/szhr/coze-communitation2/ros_test/src/music_client/srv/playmusic.srv" NAME_WE)
add_dependencies(music_client_generate_messages_py _music_client_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(music_client_genpy)
add_dependencies(music_client_genpy music_client_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS music_client_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/music_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/music_client
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(music_client_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/music_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/music_client
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(music_client_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/music_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/music_client
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(music_client_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/music_client)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/music_client
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(music_client_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/music_client)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/music_client\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/music_client
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(music_client_generate_messages_py std_msgs_generate_messages_py)
endif()
