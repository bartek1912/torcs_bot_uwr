#ifndef __QLEARNINGCONFIG_H__
#define __QLEARNINGCONFIG_H__

#include <string>
#include <vector>
using namespace std;

struct QLearningConfig
{
	// Learing rate
	float alpha;
	// Discounting factor
	float gamma;
	// Epsilon greedy parameter
	float epsilon;	
	// Track centroids
	vector<vector<float> > tracks;
	// Speed zones; Speed zones are [speeds[i], speeds[i+1])
	vector<float> speeds;
	// Steering actions 
	vector<float> steers;
	// Acceleration / Brake actions
	vector<pair<float, float> > accelBrakes;
	// Reward for getting out of the track
	float offTrackReward;
	// Reward for not moving
	float notMovingReward;
	// When we consider that the car didn't move
	float notMovingThreshold;

	bool randomStep();

	bool initialized();

	bool isTerminal(const pair<int,int>& state);

	int getActionId(const pair<int,int>& action);

	pair<int,int> getIdAction(int id);

	int getActionCount();

	friend QLearningConfig fromFile(const string& filename);
	private:
	bool _initialized;
};

QLearningConfig fromFile(const string& filename);

#endif //__QLEARNINGCONFIG_H__
