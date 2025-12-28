// Copyright (C) 2015 - 2020 Joseph Morgridge
//
// Licensed under GNU General Public License v3.0 (GPLv3) License.
// See the LICENSE file in the project root for full license details.

#include "dh_kinematic_chain.h"

#include "dh_kinematic_link.h"
#include "dh_math_utils.h"
#include "MatrixMath.h"

#if USING_ARDUINO
#include <Arduino.h>
#else
#include <iostream>
using namespace std;
#endif

namespace mt {

DhKinematicChain::DhKinematicChain(int noOflinksInput, DhKinematicLink linksInput[]) {	
	noOfLinks = noOflinksInput;
	for (int i = 0; i < noOfLinks; i++)
	{
		links[i] = linksInput[i];
	}
}

#if USING_ARDUINO
void DhKinematicChain::print_linkChain() {
	Serial.println();
	Serial.print(F("No. of links = ")); Serial.println(noOfLinks);

	for (int i = 0; i < noOfLinks; i++)
	{
		Serial.print(F("\nLink ")); Serial.print(i + 1); Serial.println(F(":"));
		links[i].print_link();
	}

	Serial.println();
}
#else
void DhKinematicChain::print_linkChain() {
	cout << endl;
	cout << "No. of links = " << noOfLinks << endl;

	for (int i = 0; i < noOfLinks; i++)
	{
		cout << "\nLink " << i + 1 << ":" << endl;
		links[i].print_link();
	}

	cout << endl;
}
#endif

#if USING_ARDUINO
void DhKinematicChain::print_qCurrent() {
	Serial.println();
	MatrixObj.Print((float*)qCurrent, 1, noOfLinks, "qCurrent = ");
	Serial.println();
}
#else
void DhKinematicChain::print_qCurrent() {
	cout << endl;
	MatrixObj.Print((float*)qCurrent, 1, 4, "qCurrent = ");
	cout << endl;
}
#endif

#if USING_ARDUINO
void DhKinematicChain::print_TmCurrent() {
	Serial.println();
	MatrixObj.Print((float*)TmCurrent, 4, 4, "TmCurrent = ");
	Serial.println();
}
#else
void DhKinematicChain::print_TmCurrent() {
	cout << endl;
	MatrixObj.Print((float*)TmCurrent, 4, 4, "TmCurrent = ");
	cout << endl;
}
#endif

int DhKinematicChain::get_noOfLinks() { return noOfLinks; }

void DhKinematicChain::get_links(DhKinematicLink linksOutput[]) {
	for (int i = 0; i < noOfLinks; i++)
	{
		linksOutput[i] = links[i];
	}
}

void DhKinematicChain::set_qCurrent(float qCurrentInput[]) {
	for (int i = 0; i < noOfLinks; i++)
	{
		qCurrent[i] = qCurrentInput[i]; // (rad).
	}

	fKineWithBaseAndTool(); // Calculate/update TmCurrent.
}

void DhKinematicChain::set_qCurrentValue(int index, float qValue) {
	qCurrent[index] = qValue; // (rad).
	fKineWithBaseAndTool(); // Calculate/update TmCurrent.
}

void DhKinematicChain::get_qCurrent(float qCurrentOutput[]) {
	for (int i = 0; i < noOfLinks; i++)
	{
		qCurrentOutput[i] = qCurrent[i]; // (rad).
	}
}

float DhKinematicChain::get_qCurrentValue(int index) { 
	return qCurrent[index]; // (rad).
}

void DhKinematicChain::get_TmCurrent(float TmCurrentOutput[4][4]) {
	MatrixObj.Copy((float*)TmCurrent, 4, 4, (float*)TmCurrentOutput);
}

void DhKinematicChain::set_TmCurrentPosition(float pxInput, float pyInput, float pzInput) {
	TmCurrent[i1][i4] = pxInput;
	TmCurrent[i2][i4] = pyInput;
	TmCurrent[i3][i4] = pzInput;
	TmCurrent[i4][i4] = 1.0;
}

void DhKinematicChain::get_TmCurrentPosition(float TmCurrentPosOutput[3]) {
	TmCurrentPosOutput[i1] = TmCurrent[i1][i4];
	TmCurrentPosOutput[i2] = TmCurrent[i2][i4];
	TmCurrentPosOutput[i3] = TmCurrent[i3][i4];
}

void DhKinematicChain::set_TmCurrentOrientation(float thetaX, float thetaY, float thetaZ, int order) {
	// Back up the position vector.
	float px = TmCurrent[i1][i4];
	float py = TmCurrent[i2][i4];
	float pz = TmCurrent[i3][i4];
	float p4 = TmCurrent[i4][i4];

	switch (order)
	{
		case 1: // X,Y,Z
		{
			DhMathUtils::trotxyz(TmCurrent, thetaX, thetaY, thetaZ);
			break;
		}

		case 2: // Z,Y,X
		{
			DhMathUtils::trotzyx(TmCurrent, thetaZ, thetaY, thetaX);
			break;
		}
	}

	// Tool is now oriented as required (BUT still located at the base frame i.e. (0, 0, 0)), hence restore the position vector.
	TmCurrent[i1][i4] = px;
	TmCurrent[i2][i4] = py;
	TmCurrent[i3][i4] = pz;
	TmCurrent[i4][i4] = p4;
}

void DhKinematicChain::multiply_TmCurrentByTm(float TmInput[4][4]) {
	float TmTemp[4][4];

	// Multiply robot T (TmCurrent) by TmInput and store result temporarily (TmTemp),
	// and copy result from temporary location (TmTemp) to robot T (TmCurrent).
	MatrixObj.Multiply((float*)TmCurrent, (float*)TmInput, 4, 4, 4, (float*)TmTemp);
	MatrixObj.Copy((float*)TmTemp, 4, 4, (float*)TmCurrent);
}

void DhKinematicChain::fKine(float TmOutput[4][4], float qInput[]) {
	float Tm[4][4] = { {1, 0, 0, 0},
										 {0, 1, 0, 0},
										 {0, 0, 1, 0},
										 {0, 0, 0, 1} };
	float TmLink[4][4];
	float TmTemp[4][4];

	for (int i = 0; i < noOfLinks; i++)
	{
		// Get transformation matrix Tm of current link (TmLink),
		// Multiply cumulated Tm by Tm of current link (TmLink) and store result temporarily (TmTemp),
		// and copy result from temporary location (TmTemp) to cumulated Tm.
		links[i].get_Tm(TmLink, qInput[i]);
		MatrixObj.Multiply((float*)Tm, (float*)TmLink, 4, 4, 4, (float*)TmTemp);
		MatrixObj.Copy((float*)TmTemp, 4, 4, (float*)Tm);
	}

	MatrixObj.Copy((float*)Tm, 4, 4, (float*)TmOutput); // Return the final result.
}

void DhKinematicChain::fKineWithBaseAndTool() {
	fKine(TmCurrent, qCurrent);
	float TmTemp[4][4];
	// Multiply robot T (TmCurrent) by tool T (TmTool) and store result temporarily (TmTemp),
	// and copy result from temporary location (TmTemp) to robot T (TmCurrent).
	MatrixObj.Multiply((float*)TmCurrent, (float*)TmTool, 4, 4, 4, (float*)TmTemp);
	MatrixObj.Copy((float*)TmTemp, 4, 4, (float*)TmCurrent);
}

void DhKinematicChain::updateTmToolInverse() {
	MatrixObj.Copy((float*)TmTool, 4, 4, (float*)TmToolInv);
	MatrixObj.Invert((float*)TmToolInv, 4);
}

void DhKinematicChain::get_TmTool(float TmToolOutput[4][4]) {
	MatrixObj.Copy((float*)TmTool, 4, 4, (float*)TmToolOutput);
}

void DhKinematicChain::get_TmToolInverse(float TmToolInverseOutput[4][4]) {
	MatrixObj.Copy((float*)TmToolInv, 4, 4, (float*)TmToolInverseOutput);
}

void DhKinematicChain::setZoffset(float zOffsetInput) {
	TmTool[i3][i4] = TmTool[i3][i4] - zOffset + zOffsetInput;
	zOffset = zOffsetInput;
	updateTmToolInverse();
	fKineWithBaseAndTool();
}

float DhKinematicChain::get_zOffset() { return zOffset; }

void DhKinematicChain::setToolTransformPosition(float dxToolInput, float dyToolInput, float dzToolInput, float zOffsetToolInput) {
	zOffset = zOffsetToolInput;
	TmTool[i1][i4] = dxToolInput;
	TmTool[i2][i4] = dyToolInput;
	TmTool[i3][i4] = dzToolInput + zOffsetToolInput;
	TmTool[i4][i4] = 1.0;
	updateTmToolInverse();
	fKineWithBaseAndTool();
}

void DhKinematicChain::setToolTransformPositionToZero() {
	zOffset = 0.0;
	TmTool[i1][i4] = 0.0;
	TmTool[i2][i4] = 0.0;
	TmTool[i3][i4] = 0.0;
	TmTool[i4][i4] = 1.0;
	updateTmToolInverse();
	fKineWithBaseAndTool();
}

} // namespace mt