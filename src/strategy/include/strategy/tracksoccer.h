#include "strategy/USinfo.h"
#include <std_msgs/String.h>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
#include "strategy/ObstacleData.h"
#include "strategy/SoccerData.h"
#include "strategy/HeadHV.h"
#include <cmath>

using namespace std;
//-------------------------------------------------

class KidsizeImage
{
public:
	KidsizeImage(ros::NodeHandle &nh)
	{
		strategy_info = StrategyInfoInstance::getInstance();
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();
		ObstacleData_Publish = nh.advertise<strategy::ObstacleData>("/tracksoccer/ObstacleData", 1000);
		SoccerData_Publish = nh.advertise<strategy::SoccerData>("/tracksoccer/SoccerData", 1000);
		HeadHV_Publish = nh.advertise<strategy::HeadHV>("/tracksoccer/HeadHV", 1000);
	};
	~KidsizeImage()
	{
		StrategyInfoInstance::deleteInstance();
		ToolInstance::deleteInstance();
		RosCommunicationInstance::deleteInstance();
	};

	StrategyInfoInstance *strategy_info;
	ToolInstance *tool;
	RosCommunicationInstance *ros_com;

	ros::Publisher ObstacleData_Publish;
	ros::Publisher SoccerData_Publish;
	ros::Publisher HeadHV_Publish;
	//---------------------------------------------
	//---------------function------------------
	void SendObstacleData();
	void SendSoccerData();
	void SendHeadHV();
	void Imagemain();
	void DrawFunction();
	void strategyclassify();
	void CheckAvoidObs();
	void MoveHead(HeadMotorID ID, int Position, int Speed);
	void FindsoccerHead();
	void TraceSoccerHead();
	void PrintTerminal();
	float IMU_Pitch_Value();

};
