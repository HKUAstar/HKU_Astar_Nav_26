execute_process(COMMAND "/home/sentry_train_test/AstarTraining/sim_nav/hdl_graph_slam/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sentry_train_test/AstarTraining/sim_nav/hdl_graph_slam/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
