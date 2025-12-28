// Copyright (C) 2015 - 2020 Joseph Morgridge
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

#ifndef DH_KINEMATIC_LINK_H_
#define DH_KINEMATIC_LINK_H_

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#define USING_ARDUINO 1
#else
#define USING_ARDUINO 0
#endif

namespace mt {

// Class to encapsulate the robot link parameters and methods.
class DhKinematicLink {
  
	// Kinematic Parameters
	float theta = 0; // Link angle.
	float d = 0; // Link offset.
	float a = 0; // Link length.
	float alpha = 0; // Link twist.

 public:

	// Constructors

	DhKinematicLink();

	DhKinematicLink(float thetaInput, float dInput, float aInput, float alphaInput);

	// Methods

	// Print link parameters.
	void print_link();

	// Get link transformation matrix.
	// Input is 4 x 4 array to store the output and angle q in rad. 
	// Output is link transformation matrix.
	void get_Tm(float TmOutput[4][4], float qInput);

	// Get link length.
	// Output is link length.
	float get_a();
};

} // namespace mt

#endif // DH_KINEMATIC_LINK_H_