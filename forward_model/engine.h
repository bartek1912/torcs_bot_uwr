#ifndef ENGINE_H_
#define ENGINE_H_
#include <vector>
using namespace std;
class Engine
{
	double rpm = 0;
	int gear;

	vector<double> rpms;
	vector<double> alpha;
	vector<double> b;
public:
	Engine();
	double getTorque(double);
	int getNewRpm(int rpm);
	void setRpm(double newRpm);
	void setGear(int newGear);
};
#endif
