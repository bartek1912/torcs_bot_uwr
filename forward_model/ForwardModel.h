#include "vector.h"
#include "matrix.h"
#include <array>
#include <vector>

struct ForwardModel
{
	linalg::vector pos, orient, vel;
	double rpm;
	int gear;
	double clutch;
	std::array<double,4> wheelSpeeds;


	//Symulacja
	double steer;
	double steerMaxSpeed;
	double steerSteerLock;

	//Lewa i prawa krawędz toru
	std::pair<std::vector<linalg::vector>, std::vector<linalg::vector> >& track;	

	ForwardModel(
			linalg::vector pos, 
			linalg::vector orient,
			double rpm, 
			linalg::vector vel, 
			int gear, 
			double clutch,
			std::array<double, 4> wheelSpeeds,
			std::pair<std::vector<linalg::vector>, std::vector<linalg::vector> >& track );

	//resetuje model zaczynajac od danej sytuacji i przywraca fizykę do domyslnych wartosci z konfiguracji
	void reset(
			linalg::vector pos,
			linalg::vector orient,
			double rpm,
			linalg::vector vel,
			int gear,
			double clutch,
			std::array<double, 4> wheelSpeeds);
	
	//inicjalizacja symulatora - stałe, pozycje kol, wektory
	void configure();
	
	//Zinterpretuj controlsy i wykonaj ruch
	void applyMove(double deltaTime, int gear, double brakes, double accel, double steer, double clutch);

	//Interpretacja controlsow
	void applyAcceleration(double accel);
	void applyGear(int gear);
	void applySteer(double steer);
	void applyBrakes(double brakes);
	void applyClutch(double clutch);

	//Zinterpretowawszy controlsy, zasymuluj fizyke
	void simulateTimestep(double deltaTime);

	//Sprawdz, czy wyjechales poza linie
	bool isOutOfTrack();
};
