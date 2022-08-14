execute_process(COMMAND "/home/max/path_plan_ws/build/GandB/third_party/arc_utilities/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/max/path_plan_ws/build/GandB/third_party/arc_utilities/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
