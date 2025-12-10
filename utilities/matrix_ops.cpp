#include "matrix_ops.h"
#include "utilities/math_ops.h" // For saturation, comparison, etc.

// NOTE: All matrix arguments are assumed to be 1D arrays (float*) representing
// data in row-major order: [a11, a12, a13, a21, a22, a23, ...]

/**
 * @brief Sets a matrix (M x N) entirely to zero.
 * @param result The matrix (1D array) to zero out.
 * @param M Number of rows.
 * @param N Number of columns.
 */
void MatrixOps::zero(float* result, int M, int N)
{
    // Simple loop to set all elements to 0.0f
    for (int i = 0; i < M * N; ++i) {
        result[i] = 0.0f;
    }
}

/**
 * @brief Sets the diagonal elements of a square matrix (N x N) to a specific value, 
 * setting all off-diagonal elements to zero.
 * @param result The square matrix (1D array).
 * @param N The dimension (rows and columns).
 * @param value The value to set the diagonal elements to.
 */
void MatrixOps::set_identity_diagonal(float* result, int N, float value)
{
    zero(result, N, N); // First, zero out the whole matrix
    for (int i = 0; i < N; ++i) {
        result[i * N + i] = value; // Accesses the element at (i, i)
    }
}


/**
 * @brief Performs matrix addition: C = A + B.
 * @param A Matrix A (M x N).
 * @param B Matrix B (M x N).
 * @param result Matrix C (M x N).
 * @param M Number of rows.
 * @param N Number of columns.
 */
void MatrixOps::add(const float* A, const float* B, float* result, int M, int N)
{
    for (int i = 0; i < M * N; ++i) {
        result[i] = A[i] + B[i];
    }
}


/**
 * @brief Performs matrix multiplication: C = A * B.
 * * A (M x K), B (K x N), Result C (M x N).
 * @param A Matrix A.
 * @param B Matrix B.
 * @param result Matrix C.
 * @param M Rows of A / Result.
 * @param K Columns of A / Rows of B.
 * @param N Columns of B / Result.
 */
void MatrixOps::multiply(const float* A, const float* B, float* result, int M, int K, int N)
{
    // C[i][j] = Sum( A[i][k] * B[k][j] ) for k=1 to K
    
    for (int i = 0; i < M; ++i) { // Row of A (and C)
        for (int j = 0; j < N; ++j) { // Column of B (and C)
            float sum = 0.0f;
            for (int k = 0; k < K; ++k) { // Inner index
                // A[i][k] * B[k][j]
                sum += A[i * K + k] * B[k * N + j]; 
            }
            result[i * N + j] = sum;
        }
    }
}


/**
 * @brief Performs matrix transposition: B = A^T.
 * * A (M x N), B (N x M).
 * @param A Matrix A.
 * @param result Matrix B (A^T).
 * @param M Rows of A.
 * @param N Columns of A.
 */
void MatrixOps::transpose(const float* A, float* result, int M, int N)
{
    for (int i = 0; i < M; ++i) { // Row of A
        for (int j = 0; j < N; ++j) { // Column of A
            // A[i][j] becomes result[j][i]
            result[j * M + i] = A[i * N + j]; 
        }
    }
}


/**
 * @brief Performs matrix inversion: A_inv = A^-1. (CONCEPTUAL ONLY)
 * * This is the most complex operation and is often handled by specialized libraries.
 * * For a large EKF (15x15), high-speed inversion is critical.
 * @param A Matrix A (N x N)
 * @param result Matrix A_inv (N x N)
 * @param N Dimension.
 * @return bool Success status.
 */
bool MatrixOps::invert(const float* A, float* result, int N)
{
    // NOTE: This operation is highly computationally expensive. 
    // In a real EKF, this would use an optimized algorithm like 
    // LU Decomposition or Cholesky Decomposition, often provided by a DSP library. 
    // A simplified Gaussian Elimination or Cofactor method is too slow 
    // for a high-rate (200Hz) embedded loop with N=15.
    
    // Placeholder function call for the required operation in the EKF:
    // CMSIS_DSP::arm_mat_inverse_f32(A, result, N);

    return true; // Assume success for conceptual purposes
}
