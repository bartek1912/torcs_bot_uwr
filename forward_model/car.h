#ifndef __CAR_H__
#define __CAR_H__

struct Steer;
struct BrakeSystem;
struct Wheel;

#include "vector.h"
#include "wheel.h"
#include "steer.h"
#include "brake.h"


/* designation */
#define FRNT_RGT	0	/**< front right */
#define FRNT_LFT	1	/**< front left */
#define REAR_RGT	2	/**< rear right */
#define REAR_LFT	3	/**< rear left */
#define FRNT		0	/**< front */
#define REAR		1	/**< rear */
#define RIGHT		0	/**< right */
#define LEFT		1	/**< left */

#define RELAXATION2(target, prev, rate) 			\
do {								\
    tdble __tmp__;						\
    __tmp__ = target;						\
    target = (prev) + (rate) * ((target) - (prev)) * 0.01;	\
    prev = __tmp__;						\
} while (0)

#define SIGN(x)     ((x) < 0 ? -1.0 : 1.0)	


struct Car
{
	linalg::transform pos;
	linalg::transform vel;
	linalg::transform acc;

	Wheel* wheels[4];
	Steer* steer;
	BrakeSystem* brakeSystem;

	double mass;
	linalg::vector inertia;

	double wheelbase;
	double wheeltrack;

	Car();

	void updateAcceleration(double deltaTime);
	void updateVelocity(double deltaTime);
	void updatePosition(double deltaTime);
	void updatePhysics(double deltaTime);
};



#endif //__CAR_H__
