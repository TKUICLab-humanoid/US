#include "strategy/USinfo.h"

USInfo* USinfo = new USInfo;

USInfo::USInfo()
{
	//--------------IMU----------------
	IMU_Yaw_initial = 0;
	IMU_Pitch_initial = 0;
	

	//----------------tracksoccer---------------------------
	//------int------
	avoidobsflag = false;
	HeadState = 0;
	HeadTurnSide = 0;
	HeadTurnSpeed = 40;
	HeadHorizontalIni = 2044;
	HeadVerticalIni = 1300;
	HorizontalMaxAngle = 2700;
	HorizontalMinAngle = 1320;
	HorizontalRange = 40;
	KickSoccerHeadVertical = 1252;
	VerticalMaxAngle = 1880;
	VerticalMinAngle = 1200;
	VerticalRange = 60;
	RobotVisionHeight = 240;
	RobotVisionWidth = 320;
	soccorcnt = 0;
	SoccerHortionalBaseLine = 120;
	SoccerVerticalBaseLine = 160;
	//------double------
	SoccerMoveX = 0;
	SoccerMoveY = 0;
	ImgHorizontalAngle = 70.42;
	ImgVerticalAngle = 43.3;
	MotorMoveOneDegree = 11.3777777777777777777777777777777;

	//------------------------strategymain-------------------------------
	//------bool------
	ContinuousFlag = false;
	ContinuousLRotationFlag = false;
	ContinuousRRotationFlag = false;

}

USInfo::~USInfo()
{

}

void USInfo::Initialize_tracksoccer()
{
	avoidobsflag = false;
	soccorcnt = 0;
	seesoccer = false;
	seeobsflag = false;
	
	Initialize_Obstacle();
	Initialize_Soccer();
}

void USInfo::Initialize_Obstacle()
{
	obstacle.X = -1;
	obstacle.Y = -1;
	obstacle.XMin = -1;
	obstacle.YMin = -1;
	obstacle.XMax = -1;
	obstacle.YMax = -1;
	obstacle.Width = -1;
	obstacle.Height = -1;
	obstacle.size = 0;
}

void USInfo::Initialize_Soccer()
{
	soccer.X = -1;
	soccer.Y = -1;
	soccer.XMin = -1;
	soccer.YMin = -1;
	soccer.XMax = -1;
	soccer.YMax = -1;
	soccer.Width = -1;
	soccer.Height = -1;
	soccer.size = 0;
}
