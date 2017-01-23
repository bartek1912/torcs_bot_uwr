#ifndef ENGINE_H_
#define ENGINE_H_
#include <vector>
using namespace std;
class Engine
{
	vector<double> rpms = {104.72, 209.44, 314.159, 418.879, 523.599, 628.319, 733.038, 837.758, 942.478, 1047.2};
	vector<double> alpha = {0.572958, 0.286479, 0.859437, 0.668451, 0.525211, 0.362873, 0.210084, 0.171887, -0.649352, -0.525211};
	vector<double> b = {100, 130, 10, 70, 130, 215, 311, 339, 1027, 910};
public:
	double getTorque(double, int, double);
	int getNewRpm(int rpm);
};
#endif