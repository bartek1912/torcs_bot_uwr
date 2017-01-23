#ifndef TRACKMODEL_H_
#define TRACKMODEL_H_
#include <vector>
#include <bits/stdc++.h>

std::pair<double, double> operator - (std::pair<double, double> a, std::pair<double, double> b);
std::ostream& operator <<(std::ostream& o, std::pair<double, double> p);
struct Track_model
{
	Track_model(){};
	void add_new_points(std::pair<double, double> left, std::pair<double, double> right)
	{
		//std::cerr<<"Pozycja "<<left<<"                       \r";
		left_edge.push_back(left);
		right_edge.push_back(right);
	}
	bool car_on_the_track(std::pair<double, double> pos);
private:
	int iloczyn_wektorow(std::pair<double, double> a, std::pair<double, double> b)
	{
		return a.first * b.second - a.second * b.first;
	}
	std::vector<std::pair<double, double> > left_edge, right_edge;
};

#endif /*TRACKMODEL_H_*/