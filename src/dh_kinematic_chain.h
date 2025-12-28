// Copyright (C) 2015 - 2020 Joseph Morgridge
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

#ifndef DH_KINEMATIC_CHAIN_H_
#define DH_KINEMATIC_CHAIN_H_

#include "dh_kinematic_link.h"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#define USING_ARDUINO 1
#else
#define USING_ARDUINO 0
#endif

namespace mt {

// Class to encapsulate the robot serial link parameters and methods.
class DhKinematicChain {

  // General Parameters
  static const int i1 = 0, i2 = 1, i3 = 2, i4 = 3; // Convenience array access indexes.
  static const int maxLinks = 7;

  // Link Chain/Series Parameters
  int noOfLinks = 0;
  DhKinematicLink links[maxLinks]; // Links for link chain/series (1 x n). 
                                   // NOTE: This requires that a zero constructor is explicitly defined for the Link class. 
                                   // Also, not defining the size of the array here doesn't throw a compile error on Arduino but
                                   // it causes all sorts of undefined behaviour/invisible problems so DON'T DO IT!
  float qCurrent[maxLinks]; // Current absolute angular positions of the joints (i.e. w.r.t D-H 0-position NOT home position or start position).
  float TmCurrent[4][4];    // Current transformation matrix (with/without tool).

  // Tool Parameters
  float zOffset = 0;
  float TmTool[4][4] = { {1, 0, 0, 0          },
                         {0, 1, 0, 0          },
                         {0, 0, 1, 0 + zOffset},
                         {0, 0, 0, 1          } }; // Tool transformation matrix.
  float TmToolInv[4][4] = { {1, 0, 0, 0},
                            {0, 1, 0, 0},
                            {0, 0, 1, 0},
                            {0, 0, 0, 1} }; // Tool transformation matrix inverse; for use in inverse kinematics calculations.

 public:

  // Constructors

  DhKinematicChain(int noOflinksInput, DhKinematicLink linksInput[]);

  // General Methods

  // Print each of the links parameters in the chain/series.
  void print_linkChain();

  // Print current joint angles.    
  void print_qCurrent();

  // Print current transformation matrix.
  void print_TmCurrent();

  // Link Methods

  // Get number of links in the chain/series.
  // Output is no. of links.
  int get_noOfLinks();

  // Get all links in the chain/series.
  // Input is array of link objects to store output.
  // Array size must match number of links. 
  // Outputs are link objects.
  void get_links(DhKinematicLink linksOutput[]);

  // Link Chain/Series Methods

  // Set current joint angles.
  // Input is array of angles in rad.
  // Array size must match number of links.
  void set_qCurrent(float qCurrentInput[]);

  // Set a single joint angle.
  // Input is joint index and angle in rad.
  // Index must be in range 0 to (no. of links - 1).
  void set_qCurrentValue(int index, float qValue);

  // Get current joint angles.
  // Input is array to store output. 
  // Array size must match number of links.
  // Outputs are angles in rad.
  void get_qCurrent(float qCurrentOutput[]);

  // Get current joint angle of indexed/specified link.
  // Input is joint angle index. 
  // Index must be in range 0 to (no. of links - 1).
  // Output is angle in rad.
  float get_qCurrentValue(int index);

  // Get current transformation matrix.
  // Input is 4 x 4 array to store output. 
  // Output is transformation matrix.
  void get_TmCurrent(float TmCurrentOutput[4][4]);

  // Set position vector in current transformation matrix.
  // Inputs is position (x, y, z).
  // The position of the end-effector (or tool-tip if a tool is applied) is changed!. USE WITH CAUTION.
  // The same end-effector orientation is maintained. If changing the end-effector orientation is required, 
  // use set_TmCurrentOrientation(...).
  void set_TmCurrentPosition(float pxInput, float pyInput, float pzInput);

  // Get position vector in current transformation matrix.
  // Input is array to store the output. 
  // Output is position vector.
  void get_TmCurrentPosition(float TmCurrentPosOutput[3]);

  // Set orientation (rotation matrix) in current transformation matrix.
  // Inputs are angles in rad and order option.
  // Input "order" options:
  // 1: X,Y,Z
  // 2: Z,X,Y
  // This function assumes you are changing the orientation of the end-effector! (or tool-tip if a tool is applied). USE WITH CAUTION.
  // The same end-effector position is maintained. If changing the end-effector position is required, 
  // use set_TmCurrentPosition(...).
  // ---
  // It would be monotonous to input the entire rotation matrix (e.g. being sent from G-code or GUI).
  // This method allows input of successive rotations of the end-effector at a time (e.g. order 1: about x, then y, then z).
  // When using this function recall rotations and translations are done with respect to the base frame i.e. assume the link is at the 
  // base (0, 0, 0) and oriented as per the base frame in the D-H coordinate system.
  void set_TmCurrentOrientation(float thetaX, float thetaY, float thetaZ, int order);

  // Multiply current transformation matrix by specified transformation matrix.
  // Input is transformation matrix in a 4 x 4 array.
  void multiply_TmCurrentByTm(float TmInput[4][4]);

  // Calculate the forward kinematics (transformation matrix) given the joint angles.
  // Inputs are 4 x 4 array to store output and array of joint angles in rad. 
  // Output is transformation matrix.
  void fKine(float TmOutput[4][4], float qInput[]);

  // Update the forward kinematics (transformation matrix) to include the tool transformation matrix.
  void fKineWithBaseAndTool();

  // Tool Methods

  // Update inverse of tool transformation matrix.
  void updateTmToolInverse();

  // Get tool transformation matrix.
  // Input is 4 x 4 array to store output. Output is transformation matrix.
  void get_TmTool(float TmToolOutput[4][4]);

  // Get inverse of tool transformation matrix.
  // Input is 4 x 4 array to store output. 
  // Output is transformation matrix.
  void get_TmToolInverse(float TmToolInverseOutput[4][4]);

  // Set tool z-offset.
  // Input is z-offset.
  void setZoffset(float zOffsetInput);

  // Get tool z-offset.
  // Output is z-offset.
  float get_zOffset();

  // Set tool transform position. In other words, define custom tool.
  // Inputs are tool dimensions (x, y, x) and offset.
  // NOTE: Due to the way the standard D-H algorithm works, the length of the robots end link may not always
  // be accounted for in the kinematic model. This function can be used to add the dimensions of the end link. 
  // If an actual tool is attached to the end link, simply consider the end link and the tool as one body and add it's 
  // dimensions using this function.
  void setToolTransformPosition(float dxToolInput, float dyToolInput, float dzToolInput, float zOffsetToolInput);

  // Set tool transform position to (0, 0, 0). In other words, no tool (and potentially no end link!).
  // Do not use this function unless you are sure of what you are doing!
  void setToolTransformPositionToZero();
};

} // namespace mt

#endif // DH_KINEMATIC_CHAIN_H_