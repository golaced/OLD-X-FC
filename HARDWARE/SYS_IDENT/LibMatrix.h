#ifndef __LIBMATRIX_H 
#define __LIBMATRIX_H 


void matrix_init0(float* A, int m, int n);
// Initialize A Matrix (Set All Elements 0)

void matrix_copy(float* A, int m, int n, float* C);
// Matrix Copy Routine

void matrix_eye(float* A, int n);
// Initialize A Matrix to An Eye Matrix

void matrix_multiply(float* A, float* B, int m, int p, int n, float* C);
// Matrix Multiplication Routine

void matrix_multiply_k(float* A, float k, int m, int n, float* C);
// Matrix Multiplication with K Routine

void matrix_addition(float* A, float* B, int m, int n, float* C);
// Matrix Addition Routine

void matrix_minus(float* A, float* B, int m, int n, float* C);
// Matrix Minus Routine

void matrix_negate(float* A, int m, int n, float* C);
// Matrix Negate Routine

void matrix_subtraction(float* A, float* B, int m, int n, float* C);
// Matrix Subtraction Routine

void matrix_transpose(float* A, int m, int n, float* C);
// Matrix Transpose Routine



int matrix_inversion(float* A, int n, float* AInverse);
// Matrix Inversion Routine

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
#endif 

