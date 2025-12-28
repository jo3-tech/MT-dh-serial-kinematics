/*
 *  MatrixMath.h Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 */
/*
 * Modifications by Joseph Morgridge
 * Original library obtained from http://playground.arduino.cc/Code/MatrixMath (Note: The link is no longer active).
 * 11/06/2016
 * - Replaced "MatrixMath Matrix" with "MatrixMath MatrixObj" in both the .h and .cpp file to avoid issues with Arduino Due.
 * - Used the F-macro in all Serial.print functions involving text strings in the .cpp file to reduce RAM usage.
 * - Modifed the statement "Inversion failed due to singular matrix" to "Error: Matrix inversion failed due to singular matrix".
 *
 * 01/05/2020
 * - Moved method comment descriptions from the .cpp file to the .h file.
 * - Added conditional compilation and relevant methods to allow the library to be used on either Arduino, or other platforms using the C++ standard library.
 */
 
#ifndef MatrixMath_h
#define MatrixMath_h

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#define USING_ARDUINO 1
#else
#include <string>
#define USING_ARDUINO 0
using namespace std;
#endif

class MatrixMath
{
public:
	//MatrixMath();

  // Matrix Printing Routine
  // Uses tabs to separate numbers under assumption printed float width won't cause problems
	// A = input matrix (m x n)
#if USING_ARDUINO
  void Print(float* A, int m, int n, String label);
#else
	void Print(float* A, int m, int n, string label);
#endif

  //Matrix Copy Routine
	void Copy(float* A, int n, int m, float* B);

  // Matrix Multiplication Routine
	// A = input matrix (m x p)
	// B = input matrix (p x n)
	// m = number of rows in A
	// p = number of columns in A = number of rows in B
	// n = number of columns in B
	// C = output matrix = A*B (m x n)
	void Multiply(float* A, float* B, int m, int p, int n, float* C);

  // Matrix Addition Routine
	// A = input matrix (m x n)
	// B = input matrix (m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix = A+B (m x n)  
  void Add(float* A, float* B, int m, int n, float* C);

  // Matrix Subtraction Routine
	// A = input matrix (m x n)
	// B = input matrix (m x n)
	// m = number of rows in A = number of rows in B
	// n = number of columns in A = number of columns in B
	// C = output matrix = A-B (m x n)
  void Subtract(float* A, float* B, int m, int n, float* C);

  // Matrix Transpose Routine
	// A = input matrix (m x n)
	// m = number of rows in A
	// n = number of columns in A
	// C = output matrix = the transpose of A (n x m)
	void Transpose(float* A, int m, int n, float* C);

  // Matrix Scale Routine
	void Scale(float* A, int m, int n, float k);

  // Matrix Inversion Routine
  // This function inverts a matrix based on the Gauss Jordan method.
  // * Specifically, it uses partial pivoting to improve numeric stability.
  // * The algorithm is drawn from those presented in
  //	 NUMERICAL RECIPES: The Art of Scientific Computing.
  // * The function returns 1 on success, 0 on failure.
  // * NOTE: The argument is ALSO the result matrix, meaning the input matrix is REPLACED
  // A = input matrix AND result matrix
	// n = number of rows = number of columns in A (n x n)
	int Invert(float* A, int n);
};

extern MatrixMath MatrixObj;
#endif
