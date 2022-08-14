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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/max/path_plan_ws/src/GandB/third_party/arc_utilities"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/max/path_plan_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/max/path_plan_ws/install/lib/python2.7/dist-packages:/home/max/path_plan_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/max/path_plan_ws/build" \
    "/usr/bin/python" \
    "/home/max/path_plan_ws/src/GandB/third_party/arc_utilities/setup.py" \
    build --build-base "/home/max/path_plan_ws/build/GandB/third_party/arc_utilities" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/max/path_plan_ws/install" --install-scripts="/home/max/path_plan_ws/install/bin"
