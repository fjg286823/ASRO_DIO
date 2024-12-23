execute_process(COMMAND "/home/fjg/work/catkin_dio/catkin_dio/build/cv_bridge/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/fjg/work/catkin_dio/catkin_dio/build/cv_bridge/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
