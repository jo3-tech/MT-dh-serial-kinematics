// Copyright (C) 2015 - 2020 Joseph Morgridge
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

#include "dh_math_utils.h"

#include "MatrixMath.h"

#if USING_ARDUINO
#include <Arduino.h>
#else
#include <iostream>
#include <cmath>
using namespace std;
#endif

namespace mt::DhMathUtils {

void rotx(float ROutput[3][3], float theta) {
  float sint = sin(theta), cost = cos(theta);
  float R[3][3] = { {1, 0,    0    },
                    {0, cost, -sint},
                    {0, sint, cost } };
  MatrixObj.Copy((float*)R, 3, 3, (float*)ROutput);
}

void trotx(float TmOutput[4][4], float theta) {
  float sint = sin(theta), cost = cos(theta);
  float Tm[4][4] = { {1, 0,    0,     0},
                     {0, cost, -sint, 0},
                     {0, sint, cost,  0},
                     {0, 0,    0,     1} };
  MatrixObj.Copy((float*)Tm, 4, 4, (float*)TmOutput);
}

void roty(float ROutput[3][3], float theta) {
  float sint = sin(theta), cost = cos(theta);
  float R[3][3] = { {cost,  0, sint},
                    {0,     1, 0   },
                    {-sint, 0, cost} };
  MatrixObj.Copy((float*)R, 3, 3, (float*)ROutput);
}

void troty(float TmOutput[4][4], float theta) {
  float sint = sin(theta), cost = cos(theta);
  float Tm[4][4] = { {cost,  0, sint, 0},
                     {0,     1, 0,    0},
                     {-sint, 0, cost, 0},
                     {0,     0, 0,    1} };
  MatrixObj.Copy((float*)Tm, 4, 4, (float*)TmOutput);
}

void rotz(float ROutput[3][3], float theta) {
  float sint = sin(theta), cost = cos(theta);
  float R[3][3] = { {cost,  -sint, 0},
                    {sint,  cost,  0},
                    {0,     0,     1} };
  MatrixObj.Copy((float*)R, 3, 3, (float*)ROutput);
}

void trotz(float TmOutput[4][4], float theta) {
  float sint = sin(theta), cost = cos(theta);
  float Tm[4][4] = { {cost,  -sint, 0, 0},
                     {sint,  cost,  0, 0},
                     {0,     0,     1, 0},
                     {0,     0,     0, 1} };
  MatrixObj.Copy((float*)Tm, 4, 4, (float*)TmOutput);
}

void rotxyz(float ROutput[3][3], float thetaX, float thetaY, float thetaZ) {
  float c1 = cos(thetaX), c2 = cos(thetaY), c3 = cos(thetaZ);
  float s1 = sin(thetaX), s2 = sin(thetaY), s3 = sin(thetaZ);
  float R[3][3] = { {c2 * c3,                    -c2 * s3,                   s2      },
                    {(c1 * s3) + (c3 * s1 * s2), (c1 * c3) - (s1 * s2 * s3), -c2 * s1},
                    {(s1 * s3) - (c1 * c3 * s2), (c3 * s1) + (c1 * s2 * s3), c1 * c2 } };

  MatrixObj.Copy((float*)R, 3, 3, (float*)ROutput);
}

void trotxyz(float TmOutput[4][4], float thetaX, float thetaY, float thetaZ) {
  float c1 = cos(thetaX), c2 = cos(thetaY), c3 = cos(thetaZ);
  float s1 = sin(thetaX), s2 = sin(thetaY), s3 = sin(thetaZ);
  float Tm[4][4] = { {c2 * c3,                    -c2 * s3,                   s2,       0},
                     {(c1 * s3) + (c3 * s1 * s2), (c1 * c3) - (s1 * s2 * s3), -c2 * s1, 0},
                     {(s1 * s3) - (c1 * c3 * s2), (c3 * s1) + (c1 * s2 * s3), c1 * c2,  0},
                     {0,                          0,                          0,        1} };
  MatrixObj.Copy((float*)Tm, 4, 4, (float*)TmOutput);
}

void rotzyx(float ROutput[3][3], float thetaZ, float thetaY, float thetaX) {
  float c1 = cos(thetaZ), c2 = cos(thetaY), c3 = cos(thetaX);
  float s1 = sin(thetaZ), s2 = sin(thetaY), s3 = sin(thetaX);
  float R[3][3] = { {c1 * c2, (c1 * s2 * s3) - (c3 * s1), (s1 * s3) + (c1 * c3 * s2)},
                    {c2 * s1, (c1 * c3) + (s1 * s2 * s3), (c3 * s1 * s2) - (c1 * s3)},
                    {-s2,     c2 * s3,                    c2 * c3                   } };
  MatrixObj.Copy((float*)R, 3, 3, (float*)ROutput);
}

void trotzyx(float TmOutput[4][4], float thetaZ, float thetaY, float thetaX) {
  float c1 = cos(thetaZ), c2 = cos(thetaY), c3 = cos(thetaX);
  float s1 = sin(thetaZ), s2 = sin(thetaY), s3 = sin(thetaX);
  float Tm[4][4] = { {c1 * c2, (c1 * s2 * s3) - (c3 * s1), (s1 * s3) + (c1 * c3 * s2), 0},
                     {c2 * s1, (c1 * c3) + (s1 * s2 * s3), (c3 * s1 * s2) - (c1 * s3), 0},
                     {-s2,     c2 * s3,                    c2 * c3,                    0},
                     {0,       0,                          0,                          1} };
  MatrixObj.Copy((float*)Tm, 4, 4, (float*)TmOutput);
}

void transl(float TmOutput[4][4], float x, float y, float z) {
  float Tm[4][4] = { {1, 0, 0, x},
                     {0, 1, 0, y},
                     {0, 0, 1, z},
                     {0, 0, 0, 1} };
  MatrixObj.Copy((float*)Tm, 4, 4, (float*)TmOutput);
}

float atan3(float num, float denom) {
  float theta = atan2(num, denom); // (rad).
  if (theta < 0) { theta = 2.0 * pi + theta; }
  return theta; // (rad).
}

float rad2deg(float thetaRad) {
  return (thetaRad * 180.0) / pi; // (deg).
}

float deg2rad(float thetaDeg) {
  return (thetaDeg * pi) / 180.0; // (rad).
}

float euclideanDistance(float pxn, float pyn, float pzn, float px, float py, float pz) {
  float dx = pxn - px;
  float dy = pyn - py;
  float dz = pzn - pz;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

float euclideanDistanceSquared(float pxn, float pyn, float pzn, float px, float py, float pz) {
  float dx = pxn - px;
  float dy = pyn - py;
  float dz = pzn - pz;
  return (dx * dx + dy * dy + dz * dz);
}

} // namespace mt::DhMathUtils