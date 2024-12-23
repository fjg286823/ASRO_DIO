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

echo_and_run cd "/home/fjg/work/catkin_dio/catkin_dio/src/cv_bridge"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/fjg/work/catkin_dio/catkin_dio/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/fjg/work/catkin_dio/catkin_dio/install/lib/python3/dist-packages:/home/fjg/work/catkin_dio/catkin_dio/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/fjg/work/catkin_dio/catkin_dio/build" \
    "/usr/bin/python3" \
    "/home/fjg/work/catkin_dio/catkin_dio/src/cv_bridge/setup.py" \
    egg_info --egg-base /home/fjg/work/catkin_dio/catkin_dio/build/cv_bridge \
    build --build-base "/home/fjg/work/catkin_dio/catkin_dio/build/cv_bridge" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/fjg/work/catkin_dio/catkin_dio/install" --install-scripts="/home/fjg/work/catkin_dio/catkin_dio/install/bin"
