#include "main.h"
#include <math.h>
#include <structures.h>
#include <funksjoner.h>
#include <string.h>
#include <stdlib.h>
#include <testfunksjoner.h>
//#include <variabler.h>
#include <variabler_ext.h>



Matrix create_matrix_from_array(int rows, int cols, float *array) {
    Matrix mat;
    mat.rows = rows;
    mat.cols = cols;

    // Allocate pointer array for rows
    mat.data = (float**) malloc(rows * sizeof(float*));
    if (mat.data == NULL) {
//        fprintf(stderr, "Memory allocation failed for data pointers.\n");
        exit(EXIT_FAILURE);
    }

    // Allocate a contiguous memory block for matrix data
    float *block = (float*) malloc(rows * cols * sizeof(float));
    if (block == NULL) {
        free(mat.data);  // Clean up previously allocated memory
//        fprintf(stderr, "Memory allocation failed for matrix data.\n");
        exit(EXIT_FAILURE);
    }

    // Copy data from the input array to the block
    memcpy(block, array, rows * cols * sizeof(float));

    // Point each row pointer to the correct position in the block
    for (int i = 0; i < rows; i++) {
        mat.data[i] = block + i * cols;
    }

    return mat;
}

void assign_matrix_data(Matrix *mat, int rows, int cols, float *array) {
    if (mat == NULL) {
//        fprintf(stderr, "Matrix pointer is NULL.\n");
        return;
    }

    // Free any existing data if necessary
    if (mat->data != NULL) {
        if (mat->data[0] != NULL) {
            free(mat->data[0]);  // Free the block of matrix data if it exists
        }
        free(mat->data);  // Free the array of pointers if it exists
    }

    mat->rows = rows;
    mat->cols = cols;

    // Allocate pointer array for rows
    mat->data = (float**) malloc(rows * sizeof(float*));
    if (mat->data == NULL) {
//        fprintf(stderr, "Memory allocation failed for data pointers.\n");
        exit(EXIT_FAILURE);
    }

    // Allocate a contiguous memory block for matrix data
    float *block = (float*) malloc(rows * cols * sizeof(float));
    if (block == NULL) {
        free(mat->data);  // Clean up previously allocated memory
//        fprintf(stderr, "Memory allocation failed for matrix data.\n");
        exit(EXIT_FAILURE);
    }

    // Copy data from the input array to the block
    memcpy(block, array, rows * cols * sizeof(float));

    // Point each row pointer to the correct position in the block
    for (int i = 0; i < rows; i++) {
        mat->data[i] = block + i * cols;
    }
}

void free_matrix(Matrix *mat) {
    if (mat->data != NULL) {
        free(mat->data[0]);  // Free the block of matrix data
        free(mat->data);     // Free the array of pointers
        mat->data = NULL;
    }
}
