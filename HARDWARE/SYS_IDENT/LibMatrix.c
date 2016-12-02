/*********************************************************************************
*                                山猫飞控（Lynx）
*                                   测试版
*
* Version   	: V1.0
* By        	: Lynx@ustc 84693469@qq.com
*
* For       	: Stm32f103VET6
* Mode      	: Thumb2
* Description   : 来源于网络并略有改动的矩阵库
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
#include <math.h>
#include <stdlib.h> //常用的函数如malloc()、calloc()、realloc()、free()、system()、atoi()、atol()、rand()、srand()、exit()等等。
#include "LibMatrix.h"
  //--------------------
	
void matrix_init0(float* A, int m, int n)
// Initialize A Matrix (Set All Elements 0)
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            A[n*i+j] = 0;
}

void matrix_copy(float* A, int m, int n, float* C)
// Matrix Copy Routine
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = A (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j];
}

void matrix_eye(float* A, int n)
// Initialize A Matrix to An Eye Matrix
{
    // A = input matrix (m x n)
    // n = number of columns and rows in A
    int i, j;
    for (i=0;i<n;i++)
        for(j=0;j<n;j++){
					if(i == j)
						A[n*i+j] = 1;
					else
						A[n*i+j] = 0;
				}
}
	
 void matrix_multiply(float* A, float* B, int m, int p, int n, float* C)
// Matrix Multiplication Routine
{
    // A = input matrix (m x p)
    // B = input matrix (p x n)
    // m = number of rows in A
    // p = number of columns in A = number of rows in B
    // n = number of columns in B
    // C = output matrix = A*B (m x n)
    int i, j, k;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
        {
          C[n*i+j]=0;
          for (k=0;k<p;k++)
            C[n*i+j]= C[n*i+j]+A[p*i+k]*B[n*k+j];
        }
}

void matrix_multiply_k(float* A, float k, int m, int n, float* C)
// Matrix Multiplication with K Routine
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = k*A (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]*k;
}

 void matrix_addition(float* A, float* B, int m, int n, float* C)
// Matrix Addition Routine
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A+B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]+B[n*i+j];
}


void matrix_minus(float* A, float* B, int m, int n, float* C)
// Matrix Minus Routine
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A-B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]-B[n*i+j];
}

void matrix_negate(float* A, int m, int n, float* C)
// Matrix Negate Routine
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = -A (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=-A[n*i+j];
}

 void matrix_subtraction(float* A, float* B, int m, int n, float* C)
// Matrix Subtraction Routine
{
    // A = input matrix (m x n)
    // B = input matrix (m x n)
    // m = number of rows in A = number of rows in B
    // n = number of columns in A = number of columns in B
    // C = output matrix = A-B (m x n)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[n*i+j]=A[n*i+j]-B[n*i+j];
}

 void matrix_transpose(float* A, int m, int n, float* C)
// Matrix Transpose Routine
{
    // A = input matrix (m x n)
    // m = number of rows in A
    // n = number of columns in A
    // C = output matrix = the transpose of A (n x m)
    int i, j;
    for (i=0;i<m;i++)
        for(j=0;j<n;j++)
            C[m*j+i]=A[n*i+j];
}


 int matrix_inversion(float* A, int n, float* AInverse)
// Matrix Inversion Routine
{
    // A = input matrix (n x n)
    // n = dimension of A
    // AInverse = inverted matrix (n x n)
    // This function inverts a matrix based on the Gauss Jordan method.
    // The function returns 1 on success, 0 on failure.
    int i, j, iPass, imx, icol, irow;
    float det, temp, pivot;
		float factor = 0;  //因为这里报错了
    float* ac = (float*)calloc(n*n, sizeof(float));
    det = 1;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            AInverse[n*i+j] = 0;
            ac[n*i+j] = A[n*i+j];
        }
        AInverse[n*i+i] = 1;
    }

    // The current pivot row is iPass.
    // For each pass, first find the maximum element in the pivot column.
    for (iPass = 0; iPass < n; iPass++)
    {
        imx = iPass;
        for (irow = iPass; irow < n; irow++)
        {
            if (fabs(A[n*irow+iPass]) > fabs(A[n*imx+iPass])) imx = irow;
        }

        // Interchange the elements of row iPass and row imx in both A and AInverse.
        if (imx != iPass)
        {
            for (icol = 0; icol < n; icol++)
            {
                temp = AInverse[n*iPass+icol];
                AInverse[n*iPass+icol] = AInverse[n*imx+icol];
                AInverse[n*imx+icol] = temp;
                if (icol >= iPass)
                {
                    temp = A[n*iPass+icol];
                    A[n*iPass+icol] = A[n*imx+icol];
                    A[n*imx+icol] = temp;
                }
            }
        }

        // The current pivot is now A[iPass][iPass].
        // The determinant is the product of the pivot elements.
        pivot = A[n*iPass+iPass];
        det = det * pivot;
        if (det == 0)
        {
            free(ac);
            return 0;
        }

        for (icol = 0; icol < n; icol++)
        {
            // Normalize the pivot row by dividing by the pivot element.
            AInverse[n*iPass+icol] = AInverse[n*iPass+icol] / pivot;
            if (icol >= iPass) A[n*iPass+icol] = A[n*iPass+icol] / pivot;
        }

        for (irow = 0; irow < n; irow++)
        {
            // Add a multiple of the pivot row to each row.  The multiple factor
            // is chosen so that the element of A on the pivot column is 0.
            if (irow != iPass) factor = A[n*irow+iPass];
            for (icol = 0; icol < n; icol++)
            {
                if (irow != iPass)
                {
                    AInverse[n*irow+icol] -= factor * AInverse[n*iPass+icol];
                    A[n*irow+icol] -= factor * A[n*iPass+icol];
                }
            }
        }
    }

    free(ac);
    return 1;
}

//static void matrix_print(float* A, int m, int n)
//// Matrix print.
//{
//    // A = input matrix (m x n)
//    // m = number of rows in A
//    // n = number of columns in A
//    int i, j;
//    for (i=0;i<m;i++)
//    {
//        printf("| ");
//        for(j=0;j<n;j++)
//        {
//            printf("%7.3f ", A[n*i+j]);
//        }
//        printf("|\n");
//    }
//}

//--------------------------------------------

