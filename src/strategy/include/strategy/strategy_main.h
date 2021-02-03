#include "strategy/USinfo.h"
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
#include "strategy/ObstacleData.h"
#include "strategy/SoccerData.h"
#include "strategy/HeadHV.h"
#include <ctime>

using namespace std;
//-------------------------------------------------

class KidsizeStrategy
{
public:
	KidsizeStrategy(ros::NodeHandle &nh)
	{
		strategy_info = StrategyInfoInstance::getInstance();
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();
		ObstacleData_subscribe = nh.subscribe("/tracksoccer/ObstacleData", 1, &KidsizeStrategy::GetObstacleData,this);
		SoccerData_subscribe = nh.subscribe("/tracksoccer/SoccerData", 1, &KidsizeStrategy::GetSoccerData,this);
		HeadHV_subscribe = nh.subscribe("/tracksoccer/HeadHV", 1, &KidsizeStrategy::GetHeadHV,this);

	};
	~KidsizeStrategy()
	{
		StrategyInfoInstance::deleteInstance();
		ToolInstance::deleteInstance();
		RosCommunicationInstance::deleteInstance();
	};

	StrategyInfoInstance *strategy_info;
	ToolInstance *tool;
	RosCommunicationInstance *ros_com;

	ros::Subscriber ObstacleData_subscribe;
	ros::Subscriber SoccerData_subscribe;
	ros::Subscriber HeadHV_subscribe;
	//---------------------------------------------
	//---------------function------------------
	float IMU_Yaw_Value();
	float IMU_Pitch_Value();
	void GetObstacleData(const strategy::ObstacleData &msg);
	void GetSoccerData(const strategy::SoccerData &msg);
	void GetHeadHV(const strategy::HeadHV &msg);
	void initparameterpath();
	void ReadIni_tracksoccer();
	void ReadIni_BodyActionValue();
	void strategymain();
	void Initialization();
	void gettimeofday_tstart();
	void gettimeofday_tend();
	void InitialMoveBody();
	void FindSoccerBody();
	void TraceSoccerBody();
	void AvoidObstacle();
	void AimGoal();
	
	void BodyAction(BodyActionState BodyActionSelect, int count = 1, int MaxValueX = 0, int MaxValueY = 0, int MaxValueTheta = 0, int DelayTime = 100);
	void ContinuousValseAdd(ContinuousValseStates ContinuousValseState, int Value,int MaxValue);
	void PrintTerminal();
	//---------------------------------------------
	bool InitialMoveBodyFlag = false;
	bool IMU_Ready = false;
	struct timeval tstart, tend;
	double timeuse;
	double ContinuousBigRotationTime;
	double ContinuousSmallRotationTime;
	double ContinuousRotationTime;
	int IMU_SET;
	int cannotsee;
	int BigRotationCnt;
	int SmallRotationCnt;
	int RotationCnt;
	string parameter_path = "N";
};
