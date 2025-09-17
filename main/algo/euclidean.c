
#include <math.h>
#include <stdio.h>

#include "euclidean.h"

struct euclidean *euclidean_add(struct euclidean *r, struct euclidean *a,
								struct euclidean *b)
{
	r->x = a->x + b->x;
	r->y = a->y + b->y;
	r->z = a->z + b->z;

	return r;
}

struct euclidean *euclidean_sub(struct euclidean *r, struct euclidean *a,
								struct euclidean *b)
{
	r->x = a->x - b->x;
	r->y = a->y - b->y;
	r->z = a->z - b->z;

	return r;
}

struct euclidean *euclidean_scale(struct euclidean *r, struct euclidean *v,
								  float s)
{
	r->x = s * v->x;
	r->y = s * v->y;
	r->z = s * v->z;

	return r;
}

float euclidean_dot(struct euclidean *a, struct euclidean *b)
{
	return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

struct euclidean *euclidean_cross(struct euclidean *r, struct euclidean *a,
								  struct euclidean *b)
{
	struct euclidean tmp;
	
	tmp.x = (a->y * b->z) - (a->z * b->y);
	tmp.y = (a->z * b->x) - (a->x * b->z);
	tmp.z = (a->x * b->y) - (a->y * b->x);

	*r = tmp;
	
	return r;
}

float euclidean_norm(struct euclidean *v)
{
	return sqrt((v->x * v->x) + (v->y * v->y) + (v->z * v->z));
}

struct euclidean *euclidean_normalize(struct euclidean *r, struct euclidean *v)
{
	float n = euclidean_norm(v);

	if (n == 0.0) {
		*r = *v;
		return r;
	}

	r->x = v->x / n;
	r->y = v->y / n;
	r->z = v->z / n;

	return r;
}

void euclidean_print(struct euclidean *v)
{
	printf("v: <%f, %f, %f>, |v| = %f\n", v->x, v->y, v->z,
		   euclidean_norm(v));

	return;
}
