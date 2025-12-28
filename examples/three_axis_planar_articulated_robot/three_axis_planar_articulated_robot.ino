// Copyright (C) 2015 - 2020 Joseph Morgridge
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

// Arduino example showing the inverse kinematics solution for a three axis planar articulated robot using the MT-dh-serial-kinematics library.

#include <dh_kinematic_link.h>
#include <dh_kinematic_chain.h>
#include <dh_math_utils.h>
#include <MatrixMath.h>

// The robots degrees of freedom (DOF).
constexpr int kDof = 3;

// The robots Denavit-Hartenberg (D-H) kinematic parameters (DH Kinematic Link instances).
//																                        theta  d    a   alpha
//																                        (rad) (mm) (mm) (rad)    Axis
mt::DhKinematicLink robot_links[kDof] = { mt::DhKinematicLink{0, 0,  150, 0},   // 1
														              mt::DhKinematicLink{0, 0,  100, 0},   // 2
														              mt::DhKinematicLink{0, 0,  0,   0} }; // 3

// Robot positions in radians (rad) relative to the "zero position" defined by the D-H algorithm.
// Zero position.
constexpr float kZeroPosition_rad[kDof] = {0, 0, 0};
// Soft home position.
//                                            {            60,              -60,              0}     (deg).
constexpr float kSoftHomePosition_rad[kDof] = {mt::DhMathUtils::pi/3, -mt::DhMathUtils::pi/3, 0}; // (rad).

// The robots kinematic model (D-H Kinematic Chain instance).
mt::DhKinematicChain robot_kinematic_model{kDof, robot_links};

constexpr int kBaudRate = 9600;

// The main application entry point for initialisation tasks.
void setup() {
  // Initialise the Serial Port.
  Serial.begin(kBaudRate);

  Serial.println(F("\n...Three axis articulated robot...\n"));

  // Display the robot links.
  Serial.println(F("...Robot links..."));
  robot_kinematic_model.print_linkChain();

  // Set the initial robot joint angles to the zero position.
  robot_kinematic_model.set_qCurrent((float*)kZeroPosition_rad); // (rad).

  // Display the robot pose/transformation matrix for the zero position.
  Serial.print(F("...Transformation matrix in the zero position..."));
  robot_kinematic_model.print_TmCurrent();

  // Set the robot joint angles to the soft home position.
  robot_kinematic_model.set_qCurrent((float*)kSoftHomePosition_rad); // (rad).

  // Display the robot pose/transformation matrix for the soft home position.
  Serial.print(F("...Transformation matrix in the home position..."));
  robot_kinematic_model.print_TmCurrent();

  // When we used set_qCurrent(...) to set the joint angles above, the fKine(...) method was called internally
  // to obtain the transformation matrix using forward kinematics, and update the robots pose. 
  // Hence we were able to display the new pose using print_TmCurrent().

  // Now, assuming we want the robot to be in a position given by cartesian coordinates (px, py).
  // We must use inverse kinematics to obtain the joint angles first before we can set the values and display
  // the new robots pose/transformation matrix as before.

  // First we manually apply the desired coordinates to the robots transformation matrix.
  // This is simply to avoid passing too many parameters to the inverse kinematics function, 
  // and we can simply pass the kinematic model (the DhKinematicChain object).
  
  const int kPx = 180; // The desired x-coordinate.
  const int kPy = 160; // The desired y-coordinate.
  robot_kinematic_model.set_TmCurrentPosition(kPx, kPy, 0);

  // Obtain the joint angles required to achieve the new robot pose.
  const int kConfiguration = 1; // Shoulder up.
  robot_inverse_kine(robot_kinematic_model, kConfiguration);

  // The robot_inverse_kine(...) internally calls set_qCurrent to update the robots pose.

  // Display the joint angles for the new robot pose.
  Serial.print(F("...Joint angles required for the new position..."));
  robot_kinematic_model.print_qCurrent();

  // Display the new robot pose/transformation matrix.
  Serial.print(F("...Transformation matrix in the new position..."));
  robot_kinematic_model.print_TmCurrent();

  Serial.println(F("\n...Robot analysis complete...\n"));
}

// The continuously running function for repetitive tasks.
void loop() {}

// Inverse kinematics solution for the three axis planar articulated robot.
// Inputs are the robots kinematic model (DhKinematicChain object), 
// with the desired transformation matrix applied (position and orientation),
// and the configuration number (1 = Shoulder up, 2 = Shoulder down). 
// Output is the array of angles in radians, stored in the input robot object.
// Equations 3-7-2, 3-7-4, 3-7-7 and 3-7-8 obtained from:
// Schilling, R.J. (1990) Fundamentals of Robotics Analysis and Control.
// Englewood Cliffs: Prentice-Hall, Inc.
void robot_inverse_kine(mt::DhKinematicChain& robot, int configuration) {

  const int i1 = 0, i2 = 1, i3 = 2, i4 = 3; // Convenience array access indexes.

  // Undo tool Transformations (if any).

  float Tm_tool_inv[4][4];
  robot.get_TmToolInverse(Tm_tool_inv);

  float Tm_input[4][4];
  robot.get_TmCurrent(Tm_input);

  float Tm_robot[4][4];
	MatrixObj.Multiply((float*)Tm_input, (float*)Tm_tool_inv, 4, 4, 4, (float*)Tm_robot);

  // Extract the required D-H parameters.

  mt::DhKinematicLink links[3];

  robot.get_links(links);

  float a1 = links[i1].get_a();
	float a2 = links[i2].get_a();

  // Extract the required Transform elements.

  // Equation 3-7-2 extract. Context given in section 3-7-4.
  float w1 = Tm_robot[i1][i4]; // P1
  float w2 = Tm_robot[i2][i4]; // P2
  float w6 = Tm_robot[i3][i3]; // R33

  // Calculate q1, q2, q3

  float q2 = acos((pow(w1, 2) + pow(w2, 2) - pow(a1, 2) - pow(a2, 2)) / (2 * a1 * a2)); // (rad). Equation 3-7-4.

  switch (configuration)
	{
    case 1: { q2 = q2; break; } // Shoulder up.
    case 2: { q2 = - q2; break; } // Shoulder down.
	}

  float C2 = cos(q2);
  float S2 = sin(q2);

  float q1 = atan2(((a1 + (a2 * C2)) * w2) - (a2 * S2 * w1), ((a1 + (a2 * C2)) * w1) + (a2 * S2 * w2)); // (rad). Equation 3-7-7.

  float q3 = mt::DhMathUtils::pi * log(w6); // (rad). Equation 3-7-8.

  float q_output[3];
	q_output[i1] = q1;
	q_output[i2] = q2;
	q_output[i3] = q3;
  
  robot.set_qCurrent(q_output); // (rad).
}
