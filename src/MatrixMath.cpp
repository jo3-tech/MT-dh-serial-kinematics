/*
 *  MatrixMath.cpp Library for Matrix Math
 *
 *  Created by Charlie Matlack on 12/18/10.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 */
/*
 * Modifications by Joseph Morgridge
 * See .h file.
*/

#include "MatrixMath.h"

#define NR_END 1

#if USING_ARDUINO
#include <Arduino.h>
#else
#include <iostream>
#include <string>
#include <cmath>
using namespace std;
#endif

MatrixMath MatrixObj;			// Pre-instantiate

#if USING_ARDUINO
void MatrixMath::Print(float* A, int m, int n, String label)
{
  int i, j;
  Serial.println();
  Serial.println(label);
  for (i = 0; i < m; i++)
  {
    for (j = 0; j < n; j++)
    {
      Serial.print(A[n * i + j]);
      Serial.print("\t");
    }
    Serial.println();
  }
}
#else
void MatrixMath::Print(float* A, int m, int n, string label)
{
	int i, j;
	cout << endl;
	cout << label << endl;
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			cout << A[n * i + j];
			cout << "\t";
		}
		cout << endl;
	}
}
#endif

void MatrixMath::Copy(float* A, int n, int m, float* B)
{
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			B[n * i + j] = A[n * i + j];
		}
}

void MatrixMath::Multiply(float* A, float* B, int m, int p, int n, float* C)
{
	int i, j, k;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
		{
			C[n * i + j] = 0;
			for (k = 0; k < p; k++)
				C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
		}
}

void MatrixMath::Add(float* A, float* B, int m, int n, float* C)
{
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] + B[n * i + j];
}

void MatrixMath::Subtract(float* A, float* B, int m, int n, float* C)
{
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
			C[n * i + j] = A[n * i + j] - B[n * i + j];
}

void MatrixMath::Transpose(float* A, int m, int n, float* C)
{
	int i, j;
	for (i = 0; i < m; i++)
		for(j = 0; j < n; j++)
			C[m * j + i] = A[n * i + j];
}

void MatrixMath::Scale(float* A, int m, int n, float k)
{
	for (int i = 0; i < m; i++)
		for (int j = 0; j < n; j++)
			A[n * i + j] = A[n * i + j] * k;
}

#if USING_ARDUINO
int MatrixMath::Invert(float* A, int n)
{
	int pivrow;		// keeps track of current pivot row
	int k,i,j;		// k: overall index along diagonal; i: row index; j: col index
	int pivrows[n]; // keeps track of rows swaps to undo at end
	float tmp;		// used for finding max value and making column swaps

	for (k = 0; k < n; k++)
	{
		// find pivot row, the row with biggest entry in current column
		tmp = 0;
		for (i = k; i < n; i++)
		{
			if (abs(A[i*n+k]) >= tmp)	// 'Avoid using other functions inside abs()?'
			{
				tmp = abs(A[i*n+k]);
				pivrow = i;
			}
		}

		// check for singular matrix
		if (A[pivrow*n+k] == 0.0f)
		{
			Serial.println(F("Error: Matrix inversion failed due to singular matrix"));
			return 0;
		}

		// Execute pivot (row swap) if needed
		if (pivrow != k)
		{
			// swap row k with pivrow
			for (j = 0; j < n; j++)
			{
				tmp = A[k*n+j];
				A[k*n+j] = A[pivrow*n+j];
				A[pivrow*n+j] = tmp;
			}
		}
		pivrows[k] = pivrow;	// record row swap (even if no swap happened)

		tmp = 1.0f/A[k*n+k];	// invert pivot element
		A[k*n+k] = 1.0f;		// This element of input matrix becomes result matrix

		// Perform row reduction (divide every element by pivot)
		for (j = 0; j < n; j++)
		{
			A[k*n+j] = A[k*n+j]*tmp;
		}

		// Now eliminate all other entries in this column
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				tmp = A[i*n+k];
				A[i*n+k] = 0.0f;  // The other place where in matrix becomes result mat
				for (j = 0; j < n; j++)
				{
					A[i*n+j] = A[i*n+j] - A[k*n+j]*tmp;
				}
			}
		}
	}

	// Done, now need to undo pivot row swaps by doing column swaps in reverse order
	for (k = n-1; k >= 0; k--)
	{
		if (pivrows[k] != k)
		{
			for (i = 0; i < n; i++)
			{
				tmp = A[i*n+k];
				A[i*n+k] = A[i*n+pivrows[k]];
				A[i*n+pivrows[k]] = tmp;
			}
		}
	}
	return 1;
}
#else
int MatrixMath::Invert(float* A, int n)
{
	int pivrow = 0;		// keeps track of current pivot row
	int k, i, j;		// k: overall index along diagonal; i: row index; j: col index

	//int pivrows[n]; // keeps track of rows swaps to undo at end
	//The above only works on Arduino - dynamic memory allocation for arrays must be used.
	int* pivrows = new int[n]; //Create pivrows array dynamically.
	
	float tmp;		// used for finding max value and making column swaps

	for (k = 0; k < n; k++)
	{
		// find pivot row, the row with biggest entry in current column
		tmp = 0;
		for (i = k; i < n; i++)
		{
			if (abs(A[i * n + k]) >= tmp)	// 'Avoid using other functions inside abs()?'
			{
				tmp = abs(A[i * n + k]);
				pivrow = i;
			}
		}

		// check for singular matrix
		if (A[pivrow * n + k] == 0.0f)
		{
			cout << "Error: Matrix inversion failed due to singular matrix" << endl;

			return 0;
		}

		// Execute pivot (row swap) if needed
		if (pivrow != k)
		{
			// swap row k with pivrow
			for (j = 0; j < n; j++)
			{
				tmp = A[k * n + j];
				A[k * n + j] = A[pivrow * n + j];
				A[pivrow * n + j] = tmp;
			}
		}
		pivrows[k] = pivrow;	// record row swap (even if no swap happened)

		tmp = 1.0f / A[k * n + k];	// invert pivot element
		A[k * n + k] = 1.0f;		// This element of input matrix becomes result matrix

		// Perform row reduction (divide every element by pivot)
		for (j = 0; j < n; j++)
		{
			A[k * n + j] = A[k * n + j] * tmp;
		}

		// Now eliminate all other entries in this column
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				tmp = A[i * n + k];
				A[i * n + k] = 0.0f;  // The other place where in matrix becomes result mat
				for (j = 0; j < n; j++)
				{
					A[i * n + j] = A[i * n + j] - A[k * n + j] * tmp;
				}
			}
		}
	}

	// Done, now need to undo pivot row swaps by doing column swaps in reverse order
	for (k = n - 1; k >= 0; k--)
	{
		if (pivrows[k] != k)
		{
			for (i = 0; i < n; i++)
			{
				tmp = A[i * n + k];
				A[i * n + k] = A[i * n + pivrows[k]];
				A[i * n + pivrows[k]] = tmp;
			}
		}
	}

	delete [] pivrows; // When done, free dynamically allocated memory for pivrows.
	pivrows = NULL;    // Clear pivrows to prevent using invalid memory reference.

	return 1;
}
#endif
