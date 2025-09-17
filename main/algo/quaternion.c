
#include <math.h>
#include <stdio.h>

#include "euclidean.h"
#include "quaternion.h"
#include "mathutil.h"

struct quaternion *quaternion_from_ordered_pair(struct quaternion *r, float s,
												struct euclidean *v)
{
	struct euclidean *im_r = (struct euclidean *) &(r->b);
	
	r->a = s;
	if (v) {
		*im_r = *v;
	} else {
		r->b = 0.0;
		r->c = 0.0;
		r->d = 0.0;
	}
	
	return r;
}

void quaternion_to_ordered_pair(struct euclidean *v, float *s, struct quaternion *q)
{
	struct euclidean *im_q = (struct euclidean *) &(q->b);

	if (s) {
		*s = q->a;
	}
	
	if (v) {
		*v = *im_q;
	}

	return;
}

struct quaternion *quaternion_conjugate(struct quaternion *r, struct quaternion *q)
{
	r->a = q->a;
	r->b = -q->b;
	r->c = -q->c;
	r->d = -q->d;

	return r;
}

float quaternion_norm(struct quaternion *q)
{
	return sqrt((q->a * q->a) + (q->b * q->b) + (q->c * q->c) + (q->d * q->d));
}

struct quaternion *quaternion_normalize(struct quaternion *r, struct quaternion *q)
{
	float n = quaternion_norm(q);

	if (n == 0.0) {
		*r = *q;
		return r;
	}

	r->a = q->a / n;
	r->b = q->b / n;
	r->c = q->c / n;
	r->d = q->d / n;

	return r;
}

struct quaternion *quaternion_inverse(struct quaternion *r, struct quaternion *q)
{
	float ns = (q->a * q->a) + (q->b * q->b) + (q->c * q->c) + (q->d * q->d);

	if (ns == 0.0) {
		*r = *q;
		return r;
	}
	
	r->a = q->a / ns;
	r->b = -q->b / ns;
	r->c = -q->c / ns;
	r->d = -q->d / ns;

	return r;
}

struct quaternion *quaternion_add(struct quaternion *r, struct quaternion *q,
								  struct quaternion *p)
{
	r->a = q->a + p->a;
	r->b = q->b + p->b;
	r->c = q->c + p->c;
	r->d = q->d + p->d;

	return r;
}

struct quaternion *quaternion_sub(struct quaternion *r, struct quaternion *q,
								  struct quaternion *p)
{
	r->a = q->a - p->a;
	r->b = q->b - p->b;
	r->c = q->c - p->c;
	r->d = q->d - p->d;

	return r;	
}

struct quaternion *quaternion_mult(struct quaternion *r, struct quaternion *q,
								   struct quaternion *p)
{
	struct quaternion tmp;
	
	tmp.a = (q->a * p->a) - (q->b * p->b) - (q->c * p->c) - (q->d * p->d);
	tmp.b = (q->a * p->b) + (q->b * p->a) + (q->c * p->d) - (q->d * p->c);
	tmp.c = (q->a * p->c) - (q->b * p->d) + (q->c * p->a) + (q->d * p->b);
	tmp.d = (q->a * p->d) + (q->b * p->c) - (q->c * p->b) + (q->d * p->a);

	*r = tmp;
	
	return r;
}

struct quaternion *quaternion_scale(struct quaternion *r, struct quaternion *q,
									float s)
{
	r->a = q->a * s;
	r->b = q->b * s;
	r->c = q->c * s;
	r->d = q->d * s;

	return r;
}

struct quaternion *quaternion_frame_transform(struct quaternion *r,
											  struct quaternion *q,
											  struct quaternion *v)
{
	struct quaternion q_inv;

	return quaternion_mult(r, quaternion_mult(r, q, v),
						   quaternion_inverse(&q_inv, q));

}
	
struct quaternion *quaternion_from_rotation_vector(struct quaternion *q,
												   struct rotation_vector *r)
{
	struct euclidean e;
	struct euclidean *im_q = (struct euclidean *) &(q->b);
	float a, ca, sa;

	rotation_vector_to_axis_angle(&e, &a, r);

	ca = cos(a / 2.0);
	sa = sin(a / 2.0);
	
	q->a = ca;
	euclidean_scale(im_q, &e, sa);

	return q;
}

struct rotation_vector *quaternion_to_rotation_vector(struct rotation_vector *r,
													  struct quaternion *q)
{
	struct euclidean *im_q = (struct euclidean *) &(q->b);
	float a;

	a = 2.0 * acos(q->a);
	
	return rotation_vector_from_axis_angle(r, im_q, a);
}

void quaternion_print(struct quaternion *q)
{
	printf("q: <%f, %f, %f, %f>, |q| = %f\n", q->a, q->b, q->c, q->d,
		   quaternion_norm(q));

	return;
}
