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

const double G = 9.80665f; /**< m/s/s */

/* conversion */
#define RADS2RPM(x) ((x)*9.549296585)		/**< Radian/s to RPM conversion */
#define RPM2RADS(x) ((x)*.104719755)		/**< RPM to Radian/s conversion */
#define RAD2DEG(x)  ((x)*(180.0/PI))		/**< Radian to degree conversion */
#define DEG2RAD(x)  ((x)*(PI/180.0))		/**< Degree to radian conversion */
#define FEET2M(x)   ((x)*0.304801)		/**< Feet to meter conversion */
#define SIGN(x)     ((x) < 0 ? -1.0 : 1.0)	/**< Sign of the expression */



#ifndef DIST
/** Distance between two points */
#define DIST(x1, y1, x2, y2) sqrt(((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2)))
#endif

#ifndef MIN
/** Minimum between two values */
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

#define RELAXATION2(target, prev, rate) 			\
do {								\
    double __tmp__;						\
    __tmp__ = target;						\
    target = (prev) + (rate) * ((target) - (prev)) * 0.01;	\
    prev = __tmp__;						\
} while (0)


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

	linalg::vector dimensions;

	Car();

	friend Car* copy(Car* car);

	void updateAcceleration(double deltaTime);
	void updateVelocity(double deltaTime);
	void updatePosition(double deltaTime);

	//To jest taki jakby interfejs z ktorego sie powinno korzystac
	//1 najpierw applyControl(...)
	//2 potem updatePhysics(...)
	//Po zrobieniu tego masz nowy stan
	void updatePhysics(double deltaTime);
	void applyControl(double deltaTime, double steer, double brake, double accel, double clutch, int gear);	
};

Car* copy(Car* car);


#endif //__CAR_H__
