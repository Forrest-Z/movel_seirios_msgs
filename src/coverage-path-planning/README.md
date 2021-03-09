# Coverage Path Planning

## Dependency Setup

This applies to noetic. If you use kinetic, use the master branch.

Source codes of packages from ipa320 have been changed to use OpenCV 4 interface (cv:: instead of CV_). If you replace these with the ones from their official repos, they would not work.

1. clone this repo outside your usual catkin workspace
1. run installation.sh to install dependencies available from apt
1. go to installation_dependencies directory
1. run `$ catkin_make install`
1. if it doesn't succeed, get all the missing packages, repeat above until it succeeds
1. copy installation_dependencies/install to your catkin_ws/install
1. copy the content of src to your catkin_ws/src
1. `source catkin_ws/install/setup.bash`
1. build your catkin workspace
