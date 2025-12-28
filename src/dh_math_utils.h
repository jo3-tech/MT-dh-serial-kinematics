// Copyright (C) 2015 - 2020 Joseph Morgridge
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

#ifndef DH_MATH_UTILS_H_
#define DH_MATH_UTILS_H_

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#define USING_ARDUINO 1
#else
#define USING_ARDUINO 0
#endif

// Namespace to encapsulate math functions.
namespace mt::DhMathUtils {

// Constants

constexpr double pi = 3.14159265358979323846;

// Functions Prototypes

// Geometry Transformations

// Rotate a right handed orthonormal coordinate system about the x-axis.
// Inputs are 3 x 3 array to store output and angle in rad. 
// Output is rotation matrix.
void rotx(float ROutput[3][3], float theta);

// Rotate a right handed orthonormal coordinate system about the x-axis.
// Inputs are 4 x 4 array to store output and angle in rad. 
// Output is transformation matrix with position vector set to 0.
void trotx(float TmOutput[4][4], float theta);

// Rotate a right handed orthonormal coordinate system about the y-axis.
// Inputs are 3 x 3 array to store output and angle in rad. 
// Output is rotation matrix.
void roty(float ROutput[3][3], float theta);

// Rotate a right handed orthonormal coordinate system about the y-axis.
// Inputs are 4 x 4 array to store output and angle in rad. 
// Output is transformation matrix with position vector set to 0.
void troty(float TmOutput[4][4], float theta);

// Rotate a right handed orthonormal coordinate system about the z-axis.
// Inputs are 3 x 3 array to store output and angle in rad. 
// Output is rotation matrix.
void rotz(float ROutput[3][3], float theta);

// Rotate a right handed orthonormal coordinate system about the z-axis.
// Inputs are 4 x 4 array to store output and angle in rad. 
// Output is transformation matrix with position vector set to 0.
void trotz(float TmOutput[4][4], float theta);

// Rotate a right handed orthonormal coordinate system about the x, y and z axes respectively.
// Inputs are 3 x 3 array to store output and angles in rad. 
// Output is rotation matrix.
void rotxyz(float ROutput[3][3], float thetaX, float thetaY, float thetaZ);

// Rotate a right handed orthonormal coordinate system about the x, y and z axes respectively.
// Inputs are 4 x 4 array to store output and angles in rad. 
// Output is transformation matrix with position vector set to 0.
void trotxyz(float TmOutput[4][4], float thetaX, float thetaY, float thetaZ);

// Rotate a right handed orthonormal coordinate system about the z, y and x axes respectively.
// Inputs are 3 x 3 array to store output and angles in rad. 
// Output is rotation matrix.
void rotzyx(float ROutput[3][3], float thetaZ, float thetaY, float thetaX);

// Rotate a right handed orthonormal coordinate system about the z, y and x axes respectively.
// Inputs are 4 x 4 array to store output and angles in rad. 
// Output is transformation matrix with position vector set to 0.
void trotzyx(float TmOutput[4][4], float thetaZ, float thetaY, float thetaX);

// Translate (move) a right handed orthonormal coordinate system along the x, y and z axes.
// Inputs are 4 x 4 array to store output and displacements (x, y, z).
// Output is transformation matrix with a unit/identity rotation matrix.
void transl(float TmOutput[4][4], float x, float y, float z);

// Trigonometry

// Remap the value of atan2 from [-pi, +pi] to [0, 2pi] rad i.e. [-180, 180] to [0, 360] deg.
// Inputs are atan2 numerator and denominator. 
// Output is angle in rad.
float atan3(float num, float denom);

// Convert radians to degrees.
// Input is angle in rad. 
// Output is angle in deg.
float rad2deg(float thetaRad);

// Convert degrees to radians.
// Input is angle in deg. 
// Output is angle in rad.
float deg2rad(float thetaDeg);

// Algebra

// Calculate the distance between two points in 3D.
// Inputs are start and end position (x, y, z). 
// Output is distance.
float euclideanDistance(float pxn, float pyn, float pzn, float px, float py, float pz);

// Calculate the square of the distance between two points in 3D.
// Inputs are start and end positions (x, y, z). 
// Output is distance^2.
float euclideanDistanceSquared(float pxn, float pyn, float pzn, float px, float py, float pz);

} // namespace mt::DhMathUtils

#endif // DH_MATH_UTILS_H_