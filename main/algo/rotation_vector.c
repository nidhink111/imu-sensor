
#include <math.h>
#include <stdio.h>

#include "const.h"
#include "euclidean.h"
#include "rotation_vector.h"

struct rotation_vector *rotation_vector_from_axis_angle(struct rotation_vector *r,
														struct euclidean *e,
														float a)
{
	struct euclidean ne;
	float na;

	// Normalize to -PI to PI
	na = fmod(a, C_TWOPI);
	na = ((float) ((int) (na / C_PI))) * -C_TWOPI + na;

	euclidean_scale((struct euclidean *)r, euclidean_normalize(&ne, &ne), na);

	return r;
}

void rotation_vector_to_axis_angle(struct euclidean *e, float *a,
								   struct rotation_vector *r)
{
	struct euclidean *er = (struct euclidean *)r;
	float rn = euclidean_norm(er);

	if (a) {
		*a = rn;
	}

	// Provide e as normalized vector, save comp time by reusing rn
	if (e) {
		if (rn == 0.0) {
			*e = *er;

			return;
		}

		e->x = r->rx / rn;
		e->y = r->ry / rn;
		e->z = r->rz / rn;
	}
}

void rotation_vector_print(struct rotation_vector *r)
{
	printf("r: <%f, %f, %f>, |r| = %f\n", r->rx, r->ry, r->rz,
		   euclidean_norm((struct euclidean *)r));

	return;
}

void axis_angle_print(struct euclidean *e, float a)
{
	printf("e: <%f, %f, %f>, a = %f, |e| = %f\n", e->x, e->y,
		   e->z, a, euclidean_norm(e));

	return;
}
