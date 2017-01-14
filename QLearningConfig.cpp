#include "QLearningConfig.h"
#include <fstream>
#include <iostream>

const int TRACK_SENSORS = 19;

bool QLearningConfig::randomStep()
{
	return (float) rand() / (((float) RAND_MAX)) < epsilon;
}

bool QLearningConfig::initialized()
{
	return _initialized;
}

bool QLearningConfig::isTerminal(const pair<int,int>& state)
{
	//Tylko track -1 -1 ... -1 jest stanem terminalnym
	for(const float& sensor : tracks[state.first])
	{
		if(sensor != -1)
			return false;
	}
	return true;
}

int QLearningConfig::getActionId(const pair<int,int>& action)
{
	return action.first * accelBrakes.size() + action.second;
}

pair<int,int> QLearningConfig::getIdAction(int id)
{
	return {id / accelBrakes.size(), id % accelBrakes.size()};
}

int QLearningConfig::getActionCount()
{
	return steers.size() * accelBrakes.size();
}

QLearningConfig fromFile(const string& filename)
{
	QLearningConfig config;
	ifstream file;
	file.open(filename, ios::in);

	while(file.peek() != ifstream::traits_type::eof())
	{
		string param;
		float val;
		file >> param >> val;
		cout << "Read " << param << " " << val << "\n";

		if(param == "END")
		{
			break;
		}
		else if(param == "ALPHA")
		{
			config.alpha = val;
		}
		else if(param == "GAMMA")
		{
			config.gamma = val;
		}
		else if(param == "EPSILON")
		{
			config.epsilon = val;
		}
		else if(param == "TRACKS")
		{
			for(int i=0;i<val;i++)
			{
				config.tracks.push_back({});
				config.tracks.back().resize(TRACK_SENSORS, -1.0);
				for(int i=0;i<TRACK_SENSORS;i++)
					file >> config.tracks.back()[i];
			}
		}
		else if(param == "SPEEDS")
		{
			for(int i=0;i<val;i++)
			{
				config.speeds.push_back(0);
				file >> config.speeds.back();
			}
		}
		else if(param == "STEERS")
		{
			for(int i=0;i<val;i++)
			{
				config.steers.push_back(0);
				file >> config.steers.back();
			}
		}
		else if(param == "ACCELBRAKES")
		{
			for(int i=0;i<val;i++)
			{
				config.accelBrakes.push_back({0,0});
				file >> config.accelBrakes.back().first >> config.accelBrakes.back().second;
			}
		}
		else if(param == "OFFTRACK_REWARD")
		{
			config.offTrackReward = val;
		}
		else if(param == "NOT_MOVING_REWARD")
		{
			config.notMovingReward = val;
		}
		else if(param == "NOT_MOVING_THRESHOLD")
		{
			config.notMovingThreshold = val;
		}
		else
		{
			throw runtime_error("Unknown parameter \"" + param + "\""); 
		}
	}

	file.close();
	config._initialized = true;
	return config;
}
