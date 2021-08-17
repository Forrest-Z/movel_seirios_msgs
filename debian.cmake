# CPACK deb generator
SET(CPACK_GENERATOR "DEB")

# populate DEBIAN/control file
## debian package name format for ros packages: ros-<rosdistro>-<package-name>
STRING(REPLACE "_" "-" DEBIAN_PACKAGE_NAME ${CMAKE_PROJECT_NAME})
SET(CPACK_PACKAGE_NAME "ros-$ENV{ROS_DISTRO}-${DEBIAN_PACKAGE_NAME}")

## get ros package version from package.xml
execute_process(
    COMMAND bash -c "cat ${CMAKE_CURRENT_SOURCE_DIR}/package.xml \\
        | grep '<version>' \\
        | perl -pe '($_)=/([0-9]+([.][0-9]+)+)/'"
    OUTPUT_VARIABLE CPACK_PACKAGE_VERSION
)

### append package version with package number and ubuntu version
find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(
    COMMAND ${LSB_RELEASE_EXEC} -cs
    OUTPUT_VARIABLE LSB_RELEASE_CODENAME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
SET(PACKAGE_NUMBER 0)
SET(CPACK_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION}-${PACKAGE_NUMBER}${LSB_RELEASE_CODENAME})

## get package description from package.xml description tag
execute_process(
    COMMAND bash -c "cat ${CMAKE_CURRENT_SOURCE_DIR}/package.xml \\
    | sed 's/^[ \\t]*//' \\
    | awk '{printf \"%s<br>\", $0}' \\
    | grep -ozP '(?<=<description>).*(?=</description>)' \\
    | sed 's/<br>/\\n/g'"
    OUTPUT_VARIABLE CPACK_PACKAGE_DESCRIPTION_SUMMARY
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Movel AI <contact@movel.ai>")
SET(CPACK_DEBIAN_PACKAGE_SECTION "misc")

# populate dependencies
SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
# execute_process(
#     COMMAND bash -c "cat ${CMAKE_CURRENT_SOURCE_DIR}/package.xml \\
#     | sed -n 's:.*<depend>\\(.*\\)</depend>.*:\\1:p' \\
#     | sed -e 's/^/ros-$ENV{ROS_DISTRO}-/' \\
#     | tr '\\n' ' ' \\
#     | tr '_' '-' \\
#     | sed -e 's/[[:space:]]*$//' \\
#     | sed -e 's/ /, /g'"
#     OUTPUT_VARIABLE ROS_DEPENDENCIES
# )
# SET(CPACK_DEBIAN_PACKAGE_DEPENDS "${ROS_DEPENDENCIES}")

# deb install configuration
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/ros/$ENV{ROS_DISTRO}")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA ${CMAKE_CURRENT_BINARY_DIR}/debian/postinst)
INCLUDE(CPack)
