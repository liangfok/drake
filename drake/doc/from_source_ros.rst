.. _build_from_source_using_ros_catkin:

**************************************
Building Drake as a ROS Catkin Package
**************************************

.. _drake_catkin_select_platform:

Please select your platform:

* :ref:`Ubuntu 16.04 (Xenial) with ROS Kinetic <build_ros_kinetic>`_
* :ref:`Ubuntu 14.04 (Trusty) with ROS Indigo <build_ros_indigo>`_

.. _drake_catkin_run_unit_tests:

How To Run Unit Tests
=====================

Execute the following commands to run Drake's ROS-based unit tests::

    cd ~/dev/drake_catkin_workspace
    source devel/setup.bash
    catkin build --verbose --no-deps drake_ros_systems --make-args run_tests

To run a single unit test like ``ros_test.test`` within package
``drake_ros_systems``, execute::

    rostest drake_ros_systems ros_test.test

To run Drake's non-ROS-based unit tests, execute::

    cd ~/dev/drake_catkin_workspace/build/drake/drake
    ctest

For more information about how to run Drake's non-ROS unit tests, see
:ref:`unit-test-instructions`.


.. _drake_catkin_additional_notes:

Additional Notes
================

.. _drake_catkin_build_documenation:

Building Drake's Documentation
------------------------------

To build Drake's documentation, execute::

    cd ~/dev/drake_catkin_workspace/build/drake/drake
    make documentation

The documentation will be located in
``~/dev/drake_catkin_workspace/build/drake/drake/doc``.

.. _drake_catkin_ci_documenation:

Scheduling a Drake / ROS Continuous Integration Test
----------------------------------------------------

Drake's Jenkin's Continuous Integration (CI) pre-merge test matrix currently
does not include a Drake + ROS column. Thus, if you change Drake's source
code and want to know whether it breaks the Drake + ROS integration, you must
manually schedule a test by posting the following comment in your PR::

    @drake-jenkins-bot linux-gcc-experimental-ros please

The command above will schedule a Drake + ROS CI pre-merge test called
"`linux-gcc-experimental-ros`". As indicated by its name, this uses the `gcc`
compiler. Links to the results are available on the PR's web page and from here:
https://drake-jenkins.csail.mit.edu/view/Experimental/job/linux-gcc-experimental-ros/.

To test the Drake + ROS integration using the `clang` compiler, post the
following comment in your PR::

    @drake-jenkins-bot linux-clang-experimental-ros please

The comment above will schedule a test called "`linux-clang-experimental-ros`".
Links to the results are available on the PR's web page and here:
https://drake-jenkins.csail.mit.edu/view/Experimental/job/linux-clang-experimental-ros/.

To schedule a full test of Drake + ROS + MATLAB with `gcc`, post the following
comment on your PR::

    @drake-jenkins-bot linux-gcc-experimental-matlab-ros please

The results will be available here:
https://drake-jenkins.csail.mit.edu/view/Experimental/job/linux-gcc-experimental-matlab-ros/.