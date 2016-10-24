.. _build_ros_kinetic:

*****************************************************************
Building Drake as a Catkin Package On Ubuntu 14.04 and ROS Indigo
*****************************************************************

.. _drake_catkin_prerequisites:

Step 0: Install Prerequisites
=============================

Install
`Ubuntu 14.04.4 LTS (Trusty Tahr) <http://releases.ubuntu.com/14.04/>`_ and
`ROS Indigo <http://wiki.ros.org/indigo>`_. Other versions of the OS and ROS
may or may not work.

Install package ``ros-indigo-ackermann-msgs``, which is used by
software in `drake_ros_systems <https://github.com/RobotLocomotion/drake/tree/master/ros/drake_ros_systems>`_::

    sudo apt-get install ros-indigo-ackermann-msgs

Once the OS and ROS are installed, install
`Catkin Tools <http://catkin-tools.readthedocs.io/en/latest/>`_ by following
the instructions
`here <http://catkin-tools.readthedocs.io/en/latest/installing.html>`_.
We use ``catkin_tools`` due to its better support building pure CMake packages
alongside Catkin packages.

.. _drake_catkin_create_workspace_directories:

Step 1: Create Directories for Holding a ROS Catkin Workspace
=============================================================

This assumes you want to create a *new* ROS workspace
in ``~/dev`` called ``drake_catkin_workspace``. The commands below can be
customized with a different workspace name and location.

If you already have a ROS Catkin workspace and simply want to add Drake as a
package within it, you can skip this step and go straight to
:ref:`drake_catkin_add_repos`.

Execute the following commands to create a directory structure for holding the
ROS workspace::

    mkdir -p ~/dev/drake_catkin_workspace/src

.. _drake_catkin_add_repos:

Step 2: Add ``drake`` and ``drake_ros_integration`` to the Workspace
====================================================================

Add ``drake`` and ``drake_ros_integration`` to the workspace::

    cd ~/dev/drake_catkin_workspace/src
    git clone git@github.com:RobotLocomotion/drake.git
    ln -s drake/ros drake_ros_integration

Note that ``drake_ros_integration`` is a symbolic link. This allows us to keep
everything in Drake's main repository without needing to completely reorganize
the files in it.

.. _drake_catkin_build_workspace:

Step 3: Build the Workspace
===========================

Execute the following commands to build the workspace::

    cd ~/dev/drake_catkin_workspace
    source /opt/ros/indigo/setup.bash
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo
    catkin build

There are numerous optional build flags that can be specified with the
``catkin config`` command listed above. These options can also be passed
directly to ``catkin build``, though they will not persist.
For a full list, execute::

    catkin config --help

The example above includes the ``cmake`` flag that specifies a build type of
``RelWithDebInfo``. Alternatives include ``Release`` and ``Debug``. Another
``cmake`` flag is ``-DDISABLE_MATLAB=TRUE``, which
disables MATLAB support. This may be useful if you have MATLAB installed but
don't have access to a license server. There are many additional command line
flags that, for example, enables support for certain optimizers like
`SNOPT <http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm>`_.
For a full list of these optional command line flags, see the variables defined
in ``~/dev/drake_catkin_workspace/src/drake/CMakeLists.txt``.

Note also that Catkin by default performs a multi-threaded build.
If your computer does not have sufficient computational resources to support
this, you can add a ``-j1`` flag after the ``catkin config`` command to force a
single-threaded build, which uses fewer resources.

Later, if you want to do a clean build, you can execute::

    cd ~/dev/drake_catkin_workspace
    catkin clean
    cd src/drake
    rm -rf externals
    git reset --hard HEAD
    git clean -fdx
    cd ~/dev/drake_catkin_workspace
    source /opt/ros/indigo/setup.bash
    catkin build
