#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/sentry_train_test/AstarTraining/sim_nav/src/hdl_graph_slam"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sentry_train_test/AstarTraining/sim_nav/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sentry_train_test/AstarTraining/sim_nav/install/lib/python3/dist-packages:/home/sentry_train_test/AstarTraining/sim_nav/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sentry_train_test/AstarTraining/sim_nav" \
    "/usr/bin/python3" \
    "/home/sentry_train_test/AstarTraining/sim_nav/src/hdl_graph_slam/setup.py" \
    egg_info --egg-base /home/sentry_train_test/AstarTraining/sim_nav/hdl_graph_slam \
    build --build-base "/home/sentry_train_test/AstarTraining/sim_nav/hdl_graph_slam" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sentry_train_test/AstarTraining/sim_nav/install" --install-scripts="/home/sentry_train_test/AstarTraining/sim_nav/install/bin"
