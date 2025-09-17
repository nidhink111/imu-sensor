
#ifndef __MADGWICK_H__
#define __MADGWICK_H__

#include "euclidean.h"
#include "quaternion.h"

struct madgwick {
	float             beta;
	struct quaternion q_est_t;
};

struct madgwick *madgwick_update(struct madgwick *mw, struct euclidean *s_m_t,
								 struct euclidean *s_a_t, struct euclidean *s_w_t,
								 float dt);

#endif /* __MADGWICK_H__ */
