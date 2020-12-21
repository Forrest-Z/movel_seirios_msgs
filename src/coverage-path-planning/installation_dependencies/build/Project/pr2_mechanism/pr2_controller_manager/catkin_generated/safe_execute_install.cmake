execute_process(COMMAND "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/pr2_mechanism/pr2_controller_manager/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/pr2_mechanism/pr2_controller_manager/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
