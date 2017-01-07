#include "track_model.h"
using namespace std;
pair<double, double> operator - (pair<double, double> a, pair<double, double> b)
{
	return make_pair(a.first - b.first, a.second - b.second);
}
std::ostream& operator <<(std::ostream& o, std::pair<double, double> p)
{
	return o<<"("<<p.first<<", "<<p.second<<") ";
}
bool Track_model::car_on_the_track(pair<double, double> pos)
{
	int sign = 0;
	for(unsigned i = 0; i + 1 < left_edge.size(); i++)
		if(iloczyn_wektorow(left_edge[i+1] - left_edge[i], pos - left_edge[i]) > 0)
		{
			if(sign == -1)
				cerr<<"Poza torem z lewej\n";
			sign = 1;
		}
		else
		{
			if(sign == 1)
				cerr<<"Poza torem z lewej\n";
			sign = -1;
		}
	sign = 0;
	for(unsigned i = 0; i + 1 < right_edge.size(); i++)
		if(iloczyn_wektorow(right_edge[i+1] - right_edge[i], pos - right_edge[i]) > 0)
		{
			if(sign == -1)
				cerr<<"Poza torem z prawej\n";
			sign = 1;
		}
		else
		{
			if(sign == 1)
				cerr<<"Poza torem prawej\n";
			sign = -1;
		}
	return false;
}