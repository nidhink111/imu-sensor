
#ifndef __ROTATION_VECTOR_H__
#define __ROTATION_VECTOR_H__

#include "euclidean.h"

/* Struct for representing rotation vectors */
struct rotation_vector {
	float rx;
	float ry;
	float rz;
};

/* Operations */
struct rotation_vector *rotation_vector_from_axis_angle(struct rotation_vector *r,
														struct euclidean *e,
														float a);
void rotation_vector_to_axis_angle(struct euclidean *e, float *a,
								   struct rotation_vector *r);

//TODO: Add more operations


void rotation_vector_print(struct rotation_vector *r);
void axis_angle_print(struct euclidean *e, float a);

#endif /* __ROTATION_VECTOR_H__ */
