
#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "euclidean.h"
#include "rotation_vector.h"

/* Struct for representing quaternions */
struct quaternion {
	float a; // 1
	float b; // i
	float c; // j
	float d; // k
};

/* Operations */
struct quaternion *quaternion_from_ordered_pair(struct quaternion *r, float s,
												struct euclidean *v);
void quaternion_to_ordered_pair(struct euclidean *v, float *s, struct quaternion *q);

struct quaternion *quaternion_conjugate(struct quaternion *r, struct quaternion *q);
float quaternion_norm(struct quaternion *q);
struct quaternion *quaternion_normalize(struct quaternion *r, struct quaternion *q);
struct quaternion *quaternion_inverse(struct quaternion *r, struct quaternion *q);

struct quaternion *quaternion_add(struct quaternion *r, struct quaternion *q,
								  struct quaternion *p);
struct quaternion *quaternion_sub(struct quaternion *r, struct quaternion *q,
								  struct quaternion *p);
struct quaternion *quaternion_mult(struct quaternion *r, struct quaternion *q,
								   struct quaternion *p);
struct quaternion *quaternion_scale(struct quaternion *r, struct quaternion *q,
									float s);

struct quaternion *quaternion_frame_transform(struct quaternion *r,
											  struct quaternion *q,
											  struct quaternion *v);

struct quaternion *quaternion_from_rotation_vector(struct quaternion *q,
												   struct rotation_vector *r);
struct rotation_vector *quaternion_to_rotation_vector(struct rotation_vector *r,
													  struct quaternion *q);


void quaternion_print(struct quaternion *q);

#endif /* __QUATERNION_H__ */
