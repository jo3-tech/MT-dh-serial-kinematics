// Copyright (C) 2015 - 2020 Joseph Morgridge
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.
 
#include "dh_kinematic_link.h"

#include "MatrixMath.h"

#if USING_ARDUINO
#include <Arduino.h>
#else
#include <iostream>
#include <cmath>
using namespace std;
#endif

namespace mt {

DhKinematicLink::DhKinematicLink():
theta(0), d(0), a(0), alpha(0) {}

DhKinematicLink::DhKinematicLink(float thetaInput, float dInput, float aInput, float alphaInput):
theta(thetaInput), d(dInput), a(aInput), alpha(alphaInput) {}

#if USING_ARDUINO
void DhKinematicLink::print_link() {
	Serial.println();
	Serial.print(F("theta = "));       Serial.print(F("q"));     Serial.print(F("\t")); 
	Serial.print(F("d = "));           Serial.print(d);          Serial.print(F("\t"));
	Serial.print(F("a = "));           Serial.print(a);          Serial.print(F("\t"));
	Serial.print(F("alpha = "));       Serial.print(alpha);      Serial.print(F("\t"));
	Serial.println();
}
#else
void DhKinematicLink::print_link() {
	cout << endl;
	cout << "theta = " << "q" << "\t";
	cout << "d = " << d << "\t";
	cout << "a = " << a << "\t";
	cout << "alpha = " << alpha << "\t";
	cout << endl;
}
#endif

void DhKinematicLink::get_Tm(float TmOutput[4][4], float qInput) {
	theta = qInput; // (rad).

	float sina = sin(alpha), cosa = cos(alpha);
	float sint = sin(theta), cost = cos(theta);

	float Tm[4][4] = { {cost, -sint * cosa, sint * sina,  a * cost},
										 {sint, cost * cosa,  -cost * sina, a * sint},
										 {0,    sina,         cosa,         d       },
										 {0,    0,            0,            1       } };

	MatrixObj.Copy((float*)Tm, 4, 4, (float*)TmOutput);
}

float DhKinematicLink::get_a() { return a; }

} // namespace mt