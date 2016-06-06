#!/bin/bash

# To run this bash script, execute:
#   $ cd [drake distro]/drake/ros/scripts/
#   $ source install_drake_ros_package.sh

# Declares the installation prefix directory.
INSTALL_PREFIX="/tmp/drake_ros"
PACKAGE_NAME="drake_ros_package"
UNIT_TEST_NAME="ros_test"

# Creates the directory structures
mkdir --parents $INSTALL_PREFIX/lib/$PACKAGE_NAME
mkdir --parents $INSTALL_PREFIX/share/$PACKAGE_NAME
mkdir --parents $INSTALL_PREFIX/share/$PACKAGE_NAME/test

# Creates the .catkin file.
touch $INSTALL_PREFIX/.catkin

# Creates a package.xml file.
echo "<package><name>$PACKAGE_NAME</name></package>" > $INSTALL_PREFIX/share/$PACKAGE_NAME/package.xml

# Creates a ros_test.test file.
echo "<launch><test test-name=\"$UNIT_TEST_NAME\" pkg=\"drake_ros_package\" type=\"$UNIT_TEST_NAME\" /></launch>" > $INSTALL_PREFIX/share/$PACKAGE_NAME/test/$UNIT_TEST_NAME.test

# Saves the $UNIT_TEST_NAME executable into the ROS catkin environment.
UNIT_TEST_EXEC_PATH=$(find ../../ -name $UNIT_TEST_NAME)
cp $UNIT_TEST_EXEC_PATH $INSTALL_PREFIX/lib/$PACKAGE_NAME

# Update the CMAKE_PREFIX_PATH and ROS_PACKAGE_PATH environment variables.
export CMAKE_PREFIX_PATH=$INSTALL_PREFIX:$CMAKE_PREFIX_PATH
export ROS_PACKAGE_PATH=$INSTALL_PREFIX/share:$ROS_PACKAGE_PATH

echo "Done installing $UNIT_TEST_NAME unit test."
echo "You can run it by executing \"rostest $PACKAGE_NAME $UNIT_TEST_NAME.test\"."