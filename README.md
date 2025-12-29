# MT-dh-serial-kinematics

[![Static check](https://github.com/jo3-tech/MT-dh-serial-kinematics/actions/workflows/static-check.yaml/badge.svg)](https://github.com/jo3-tech/MT-dh-serial-kinematics/actions/workflows/static-check.yaml) [![Build examples](https://github.com/jo3-tech/MT-dh-serial-kinematics/actions/workflows/build-examples.yaml/badge.svg)](https://github.com/jo3-tech/MT-dh-serial-kinematics/actions/workflows/build-examples.yaml)

Kinematics library in C++ for serial manipulators/robots using the Denavit-Hartenberg (D-H) algorithm.

This legacy project (developed between 2015 and 2020) was originally created as a lightweight robotics library for the Arduino platform, however, it can also be used as a general purpose C++ library (C++11 and above) for desktop platforms. It uses conditional compilation to determine the platform (Arduino or desktop), then includes the relevant headers, uses the relevant data types, and implements the relevant methods specific to the platform.

All code, documentation, and descriptions were originally written over 5 years ago and have been copied here with only minor modification. As such, they may lack polish, clarity, or completeness, and do not reflect current best practices.

The library includes various components packaged together for convenience.

|Header|Description|
|:----|----|
|dh_kinematic_link.h|The first part of the main library for creating robot links with D-H kinematic parameters.|
|dh_kinematic_chain.h|The second part of the main library for creating the D-H kinematic model (serial chain) of the robot using the links.|
|dh_math_utils.h|A utility library containing some math functions commonly used in implementing robot kinematics (geometry transformation, trigonometry, and algebra).|
|MatrixMath.h|A lightweight matrix library originally obtained from the public domain at [Arduino Playground](http://playground.arduino.cc/Code/MatrixMath), however, the link is no longer active. The library was modified for this project. Attributions can be found in the header.|

See the [examples](examples) folder for how to get started using the library from an example showing the inverse kinematics solution for a 3-axis planar articulated robot.

The [extras](extras) folder contains images showing the inverse kinematics solution for the 3-axis planar articulated robot with the [shoulder up](extras/planar_rrr_robot_ikine_shoulder_up.png) and [shoulder down](extras/planar_rrr_robot_ikine_shoulder_down.png) configurations. It also contains a [document](extras/geometry%20transformations.pdf) describing the use of geometry transformation functions as a gentle introduction to serial chain kinematics.

This library can be installed via the Arduino Library Manager for Arduino projects. For desktop projects, simply copy the files into your project.
