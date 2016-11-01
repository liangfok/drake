#------------------------------------------------------------------------------
# Add a ROS package. Besides setting up the external project, this also creates
# the option used to control if the ROS package is enabled.
#
# Arguments:
#   <PACKAGE> - Name of the example.
#   <DEFAULT_ENABLED> - Is the example enabled by default? (`ON` or `OFF`)
#
#   <DOCSTRING>
#       Description of the ROS package, to be used as the documentation string
#       for the example's option.
#------------------------------------------------------------------------------
function(drake_add_ros_package PACKAGE DEFAULT_ENABLED DOCSTRING)
  string(TOUPPER ${PACKAGE} PACKAGE_UPPER)

  # Add option for example
  option(ROS_PACKAGE_${PACKAGE_UPPER} ${DOCSTRING} ${DEFAULT_ENABLED})

  # Set up example, if enabled
  if(ROS_PACKAGE_${PACKAGE_UPPER})
    message(STATUS "Installation will include ROS package ${PACKAGE}.")
    message(STATUS "Download command is: ${_ex_DOWNLOAD_COMMAND}.")
    message(STATUS "Update command is: ${_ex_UPDATE_COMMAND}.")
    message(STATUS "Project source is: ${PROJECT_SOURCE_DIR}.")

    # Set up submodule and commands for synchronizing submodule
    drake_add_submodule(drake/examples/${PACKAGE}
      _ex_DOWNLOAD_COMMAND _ex_UPDATE_COMMAND)

    # Add the external project
    ExternalProject_Add(download-${PACKAGE}
      SOURCE_DIR ${PROJECT_SOURCE_DIR}/ros/${PACKAGE}
      DOWNLOAD_COMMAND ${_ex_DOWNLOAD_COMMAND}
      DOWNLOAD_DIR ${PROJECT_SOURCE_DIR}
      UPDATE_COMMAND ${_ex_UPDATE_COMMAND}
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      INSTALL_COMMAND "")

    # Set up build step to ensure project is updated before build
    drake_forceupdate(download-${PACKAGE})

    # Add example to download dependencies
    add_dependencies(download-all download-${PACKAGE})
    add_dependencies(drake download-${PACKAGE})
  endif()
endfunction()
