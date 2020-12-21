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

echo_and_run cd "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_navigation/cob_map_accessibility_analysis"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/install/Project/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/install/Project/lib/python3/dist-packages:/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project" \
    "/usr/bin/python3" \
    "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/src/cob_navigation/cob_map_accessibility_analysis/setup.py" \
     \
    build --build-base "/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/build/Project/cob_navigation/cob_map_accessibility_analysis" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/install/Project" --install-scripts="/home/ryan/seirios-ros/src/movel_planning/coverage-path-planning/installation_dependencies/install/Project/bin"
