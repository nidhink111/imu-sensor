
#include <math.h>
#include "madgwick.h"
#include "euclidean.h"
#include "quaternion.h"
#include <stdio.h>

// Because you cant just pass in and derefence J for the helper functions,
// use this macro to act like J[r][c] syntax
#define _J(r, c) *(J + ((r) * 4) + (c))

static struct quaternion *f_obj(struct quaternion *r, struct quaternion *se_q,
								struct quaternion *e_d, struct quaternion *s_s)
{
	struct quaternion qdq;
	struct quaternion se_q_conj;

	// Implement the objective function f (Eq 6)
	quaternion_frame_transform(&qdq, quaternion_conjugate(&se_q_conj, se_q),
							   e_d);
	
	return quaternion_sub(r, &qdq, s_s); 
}

static float *J_g(float *J, struct quaternion *se_q)
{
	struct quaternion q;

	// Most all elements are scaled by 2, do that first
	quaternion_scale(&q, se_q, 2.0);
	
	// Eq (13)
	_J(0, 0) = -q.c;
	_J(0, 1) = q.d;
	_J(0, 2) = -q.a;
	_J(0, 3) = q.b;

	_J(1, 0) = q.b;
	_J(1, 1) = q.a;
	_J(1, 2) = q.d;
	_J(1, 3) = q.c;

	_J(2, 0) = 0.0;
	_J(2, 1) = -2.0 * q.b;
	_J(2, 2) = -2.0 * q.c;
	_J(2, 3) = 0.0;

	return J;
}

static float *J_b(float *J, struct quaternion *se_q, struct quaternion *e_b)
{
	struct quaternion q;

	// Most all elements are scaled by 2, do that first
	quaternion_scale(&q, se_q, 2.0);

	// Eq (17)
	_J(0, 0) = -q.c * e_b->d;
	_J(0, 1) = q.d * e_b->d;
	_J(0, 2) = (-q.a * e_b->d) + (-2.0 * q.c * e_b->b);
	_J(0, 3) = (q.b * e_b->d) + (-2.0 * q.d * e_b->b);

	_J(1, 0) = (q.b * e_b->d) + (-q.d * e_b->b);
	_J(1, 1) = (q.a * e_b->d) + (q.c * e_b->b);
	_J(1, 2) = (q.d * e_b->d) + (q.b * e_b->b);
	_J(1, 3) = (q.c * e_b->d) + (-q.a * e_b->b);

	_J(2, 0) = q.c * e_b->b;
	_J(2, 1) = (-2.0 * q.b * e_b->d) + (q.d * e_b->b);
	_J(2, 2) = (-2.0 * q.c * e_b->d) + (q.a * e_b->b);
	_J(2, 3) = q.b * e_b->b;

	return J;
}

struct madgwick *madgwick_update(struct madgwick *mw, struct euclidean *s_m_t,
								 struct euclidean *s_a_t, struct euclidean *s_w_t,
								 float dt)
{
	struct quaternion s_a, s_w, s_m;
	struct quaternion e_b_t;
	struct quaternion grad_f;
	struct quaternion e_g;
	struct quaternion se_d_q;
	float J_g_b[6][4];
	float f_g_b[8];
	float *grad_f_fp;
	struct euclidean s_m_t_n;
	struct euclidean s_a_t_n;

	// Normalize magnetometer and accel samples
	euclidean_normalize(&s_m_t_n,s_m_t);
	euclidean_normalize(&s_a_t_n,s_a_t);
    
	// Convert all sensors to pure quaternions
	quaternion_from_ordered_pair(&s_a, 0, &s_a_t_n);
	quaternion_from_ordered_pair(&s_w, 0, s_w_t);
	quaternion_from_ordered_pair(&s_m, 0, &s_m_t_n);
    
	// Magnetometer distortion compensation (Eq 31,32)
	quaternion_frame_transform(&e_b_t, &(mw->q_est_t), &s_m);
    
	e_b_t.a = 0.0f;
	e_b_t.b = sqrt((e_b_t.b * e_b_t.b) + (e_b_t.c * e_b_t.c));
	e_b_t.c = 0.0f;
    
	// Calculate the gradient
	// (Eq 18,19)
	// NOTE: (Eq 19) is wrong, it SHOULD be J_g_b = [J_g ; J_b], not
	// [J_T_g ; J_T_b]!!!
	// f_g:
	e_g.a = 0.0f;
	e_g.b = 0.0f;
	e_g.c = 0.0f;
	e_g.d = 1.0f;
	
	f_obj((struct quaternion *) &(f_g_b[0]), &(mw->q_est_t), &e_g, &s_a);

	// J_g:
	J_g(&(J_g_b[0][0]), &(mw->q_est_t));

	// f_b:
	f_obj((struct quaternion *) &(f_g_b[4]), &(mw->q_est_t), &e_b_t, &s_m);

	// J_b:
	J_b(&(J_g_b[3][0]), &(mw->q_est_t), &e_b_t);

	// Eq (21)
	// Clear grad_f and setup array pointer
	grad_f.a = 0.0f;
	grad_f.b = 0.0f;
	grad_f.c = 0.0f;
	grad_f.d = 0.0f;
	grad_f_fp = (float *) &(grad_f);

	// Mat Mult:
	for (int c = 0; c < 4; c++) {
		for (int r = 0; r < 6; r++) {
			// Pick the correct elements from f_g_b (ie, 1:3,5:7)
			grad_f_fp[c] += J_g_b[r][c] * f_g_b[r + 1 + (r / 3)];
		}
	}

	// Angular rate
	// Eq (3)
	quaternion_scale(&se_d_q, quaternion_mult(&se_d_q, &(mw->q_est_t), &s_w),
					 0.5f);

	// Fusion
	// Eq (30)
	quaternion_sub(&se_d_q, &se_d_q,
				   quaternion_scale(&grad_f,
									quaternion_normalize(&grad_f, &grad_f),
									mw->beta));

	// Eq (29)
	quaternion_add(&(mw->q_est_t), &(mw->q_est_t),
				   quaternion_scale(&se_d_q, &se_d_q, dt));

	// Normialize final result
	quaternion_normalize(&(mw->q_est_t), &(mw->q_est_t));

	return mw;
}
