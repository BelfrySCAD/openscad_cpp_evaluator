// Jonathan Richard Shewchuk's adaptive exact geometric predicates
// (https://www.cs.cmu.edu/~quake/robust.html), placed in the public domain.
// predicates.c is the original with its random-number test helpers removed.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Must run once before any predicate; sets the machine epsilon and splitter.
void exactinit(void);
// Positive if d lies below the plane through a, b, c (a, b, c counterclockwise
// seen from above), negative if above, zero if coplanar. Exact sign.
double orient3d(const double* pa, const double* pb, const double* pc, const double* pd);
// Positive if e lies inside the sphere through a, b, c, d (for orient3d(a,b,c,d)
// positive), negative outside, zero on it. Exact sign.
double insphere(const double* pa, const double* pb, const double* pc, const double* pd,
                const double* pe);

#ifdef __cplusplus
}
#endif
