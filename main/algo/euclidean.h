
#ifndef __EUCLIDEAN_H__
#define __EUCLIDEAN_H__

/* Struct for representing Euclidean (spatial) vectors */
struct euclidean {
	float x;
	float y;
	float z;
};

/* Operations */
struct euclidean *euclidean_add(struct euclidean *r, struct euclidean *a,
								struct euclidean *b);
struct euclidean *euclidean_sub(struct euclidean *r, struct euclidean *a,
								struct euclidean *b);
struct euclidean *euclidean_scale(struct euclidean *r, struct euclidean *v,
								  float s);

float euclidean_dot(struct euclidean *a, struct euclidean *b);
struct euclidean *euclidean_cross(struct euclidean *r, struct euclidean *a,
								  struct euclidean *b);

float euclidean_norm(struct euclidean *v);
struct euclidean *euclidean_normalize(struct euclidean *r, struct euclidean *v);


void euclidean_print(struct euclidean *v);

#endif /* __EUCLIDEAN_H__ */
