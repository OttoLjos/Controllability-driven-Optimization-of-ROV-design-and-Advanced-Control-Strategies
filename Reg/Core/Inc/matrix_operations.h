/*
 * matrix_operations.h
 *
 *  Created on: May 16, 2024
 *      Author: trcho
 */

#ifndef INC_MATRIX_OPERATIONS_H_
#define INC_MATRIX_OPERATIONS_H_


void free_matrix(Matrix *mat);
Matrix create_matrix_from_array(int rows, int cols, float *array);
void assign_matrix_data(Matrix *mat, int rows, int cols, float *array);


#endif /* INC_MATRIX_OPERATIONS_H_ */
