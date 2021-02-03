#include "strategy/strategy_main.h"

void KidsizeStrategy::GetObstacleData(const strategy::ObstacleData &msg)
{
	USinfo->obstacle.X = msg.obstacleX;
	USinfo->obstacle.Y = msg.obstacleY;
	USinfo->obstacle.XMin = msg.obstacleXMin;
	USinfo->obstacle.XMax = msg.obstacleXMax;
	USinfo->obstacle.YMin = msg.obstacleYMin;
	USinfo->obstacle.YMax = msg.obstacleYMax;
}

void KidsizeStrategy::GetSoccerData(const strategy::SoccerData &msg)
{
	USinfo->soccer.X = msg.soccerX;
	USinfo->soccer.Y = msg.soccerY;
	USinfo->soccer.XMin = msg.soccerXMin;
	USinfo->soccer.XMax = msg.soccerXMax;
	USinfo->soccer.YMin = msg.soccerYMin;
	USinfo->soccer.YMax = msg.soccerYMax;
	USinfo->soccer.size = msg.soccersize;
	USinfo->seesoccer = msg.seesoccer;
}

void KidsizeStrategy::GetHeadHV(const strategy::HeadHV &msg)
{
	USinfo->HeadHorizontal = msg.HeadHorizontal;
	USinfo->HeadVertical = msg.HeadVertical;
}

void KidsizeStrategy::initparameterpath()
{
	while(parameter_path == "N")
	{
		parameter_path = tool->getPackagePath("strategy");
	}
	printf("parameter_path is %s\n", parameter_path.c_str());
}

void KidsizeStrategy::ReadIni_tracksoccer()
{
	fstream fin;
	string sTmp;
	char line[100];
	char path[200];
	strcpy(path, parameter_path.c_str());
	strcat(path, "/tracksoccerinfo.ini");
	fin.open(path, ios::in);
	try
	{
		ROS_INFO("openfile tracksoccerinfo.ini");

		USinfo->FixHead_LB_Limit = tool->readvalue(fin, "FixHead_LB_Limit", 0);//545
		USinfo->FixHead_RB_Limit = tool->readvalue(fin, "FixHead_RB_Limit", 0);//485
		USinfo->MiddleDecade = tool->readvalue(fin, "MiddleDecade", 0);
		USinfo->NearDecade = tool->readvalue(fin, "NearDecade", 0);
		USinfo->StartPositionError = tool->readvalue(fin, "StartPositionError", 1);
		USinfo->AimGoalFlag = tool->readvalue(fin, "AimGoalFlag", 0);
		USinfo->OneSteptoContinuousRatio = tool->readvalue(fin, "OneSteptoContinuousRatio", 1);
		USinfo->ContinuousTimeError = tool->readvalue(fin, "ContinuousTimeError", 1);
		USinfo->ContinuousTimeRatio = tool->readvalue(fin, "ContinuousTimeRatio", 1);

		ROS_INFO("fileclose tracksoccerinfo.ini");
	}
	catch (exception e)
	{
		ROS_INFO("catchfile tracksoccerinfo.ini");
	}
}

void KidsizeStrategy::ReadIni_BodyActionValue()
{
	fstream fin;
	string sTmp;
	char line[100];
	char path[200];
	char temp[100];
	strcpy(path, parameter_path.c_str());
	strcat(path, "/BodyActionValue.ini");
	fin.open(path, ios::in);
	try
	{
		ROS_INFO("openfile BodyActionValue.ini");

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[Move].X = tool->readvalue(fin, "[Move].X", 0);
		USinfo->MoveValue[Move].Y = tool->readvalue(fin, "[Move].Y", 0);
		USinfo->MoveValue[Move].theta = tool->readvalue(fin, "[Move].theta", 0);
		USinfo->MoveValue[Move].Mode = tool->readvalue(fin, "[Move].Mode", 0);
		USinfo->MoveValue[Move].IMU = tool->readvalue(fin, "[Move].IMU", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[LMove].X = tool->readvalue(fin, "[LMove].X", 0);
		USinfo->MoveValue[LMove].Y = tool->readvalue(fin, "[LMove].Y", 0);
		USinfo->MoveValue[LMove].theta = tool->readvalue(fin, "[LMove].theta", 0);
		USinfo->MoveValue[LMove].Mode = tool->readvalue(fin, "[LMove].Mode", 0);
		USinfo->MoveValue[LMove].IMU = tool->readvalue(fin, "[LMove].IMU", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[RMove].X = tool->readvalue(fin, "[RMove].X", 0);
		USinfo->MoveValue[RMove].Y = tool->readvalue(fin, "[RMove].Y", 0);
		USinfo->MoveValue[RMove].theta = tool->readvalue(fin, "[RMove].theta", 0);
		USinfo->MoveValue[RMove].Mode = tool->readvalue(fin, "[RMove].Mode", 0);
		USinfo->MoveValue[RMove].IMU = tool->readvalue(fin, "[RMove].IMU", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[SmallLRotation].X = tool->readvalue(fin, "[SmallLRotation].X", 0);
		USinfo->MoveValue[SmallLRotation].Y = tool->readvalue(fin, "[SmallLRotation].Y", 0);
		USinfo->MoveValue[SmallLRotation].theta = tool->readvalue(fin, "[SmallLRotation].theta", 0);
		USinfo->MoveValue[SmallLRotation].Mode = tool->readvalue(fin, "[SmallLRotation].Mode", 0);
		USinfo->MoveValue[SmallLRotation].IMU = tool->readvalue(fin, "[SmallLRotation].IMU", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[BigLRotation].X = tool->readvalue(fin, "[BigLRotation].X", 0);
		USinfo->MoveValue[BigLRotation].Y = tool->readvalue(fin, "[BigLRotation].Y", 0);
		USinfo->MoveValue[BigLRotation].theta = tool->readvalue(fin, "[BigLRotation].theta", 0);
		USinfo->MoveValue[BigLRotation].Mode = tool->readvalue(fin, "[BigLRotation].Mode", 0);
		USinfo->MoveValue[BigLRotation].IMU = tool->readvalue(fin, "[BigLRotation].IMU", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[SmallRRotation].X = tool->readvalue(fin, "[SmallRRotation].X", 0);
		USinfo->MoveValue[SmallRRotation].Y = tool->readvalue(fin, "[SmallRRotation].Y", 0);
		USinfo->MoveValue[SmallRRotation].theta = tool->readvalue(fin, "[SmallRRotation].theta", 0);
		USinfo->MoveValue[SmallRRotation].Mode = tool->readvalue(fin, "[SmallRRotation].Mode", 0);
		USinfo->MoveValue[SmallRRotation].IMU = tool->readvalue(fin, "[SmallRRotation].IMU", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[BigRRotation].X = tool->readvalue(fin, "[BigRRotation].X", 0);
		USinfo->MoveValue[BigRRotation].Y = tool->readvalue(fin, "[BigRRotation].Y", 0);
		USinfo->MoveValue[BigRRotation].theta = tool->readvalue(fin, "[BigRRotation].theta", 0);
		USinfo->MoveValue[BigRRotation].Mode = tool->readvalue(fin, "[BigRRotation].Mode", 0);
		USinfo->MoveValue[BigRRotation].IMU = tool->readvalue(fin, "[BigRRotation].IMU", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousStart].X = tool->readvalue(fin, "[ContinuousStart].X", 0);
		USinfo->MoveValue[ContinuousStart].Y = tool->readvalue(fin, "[ContinuousStart].Y", 0);
		USinfo->MoveValue[ContinuousStart].theta = tool->readvalue(fin, "[ContinuousStart].theta", 0);
		USinfo->MoveValue[ContinuousStart].Mode = tool->readvalue(fin, "[ContinuousStart].Mode", 0);
		USinfo->MoveValue[ContinuousStart].IMU = tool->readvalue(fin, "[ContinuousStart].IMU", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousShift].X = tool->readvalue(fin, "[ContinuousShift].X", 0);
		USinfo->MoveValue[ContinuousShift].Y = tool->readvalue(fin, "[ContinuousShift].Y", 0);
		USinfo->MoveValue[ContinuousShift].theta = tool->readvalue(fin, "[ContinuousShift].theta", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousLMove].X = tool->readvalue(fin, "[ContinuousLMove].X", 0);
		USinfo->MoveValue[ContinuousLMove].Y = tool->readvalue(fin, "[ContinuousLMove].Y", 0);
		USinfo->MoveValue[ContinuousLMove].theta = tool->readvalue(fin, "[ContinuousLMove].theta", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousRMove].X = tool->readvalue(fin, "[ContinuousRMove].X", 0);
		USinfo->MoveValue[ContinuousRMove].Y = tool->readvalue(fin, "[ContinuousRMove].Y", 0);
		USinfo->MoveValue[ContinuousRMove].theta = tool->readvalue(fin, "[ContinuousRMove].theta", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousSmallLRotation].X = tool->readvalue(fin, "[ContinuousSmallLRotation].X", 0);
		USinfo->MoveValue[ContinuousSmallLRotation].Y = tool->readvalue(fin, "[ContinuousSmallLRotation].Y", 0);
		USinfo->MoveValue[ContinuousSmallLRotation].theta = tool->readvalue(fin, "[ContinuousSmallLRotation].theta", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousBigLRotation].X = tool->readvalue(fin, "[ContinuousBigLRotation].X", 0);
		USinfo->MoveValue[ContinuousBigLRotation].Y = tool->readvalue(fin, "[ContinuousBigLRotation].Y", 0);
		USinfo->MoveValue[ContinuousBigLRotation].theta = tool->readvalue(fin, "[ContinuousBigLRotation].theta", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousSmallRRotation].X = tool->readvalue(fin, "[ContinuousSmallRRotation].X", 0);
		USinfo->MoveValue[ContinuousSmallRRotation].Y = tool->readvalue(fin, "[ContinuousSmallRRotation].Y", 0);
		USinfo->MoveValue[ContinuousSmallRRotation].theta = tool->readvalue(fin, "[ContinuousSmallRRotation].theta", 0);
		
		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousBigRRotation].X = tool->readvalue(fin, "[ContinuousBigRRotation].X", 0);
		USinfo->MoveValue[ContinuousBigRRotation].Y = tool->readvalue(fin, "[ContinuousBigRRotation].Y", 0);
		USinfo->MoveValue[ContinuousBigRRotation].theta = tool->readvalue(fin, "[ContinuousBigRRotation].theta", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousKickBallLRotation].X = tool->readvalue(fin, "[ContinuousKickBallLRotation].X", 0);
		USinfo->MoveValue[ContinuousKickBallLRotation].Y = tool->readvalue(fin, "[ContinuousKickBallLRotation].Y", 0);
		USinfo->MoveValue[ContinuousKickBallLRotation].theta = tool->readvalue(fin, "[ContinuousKickBallLRotation].theta", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousKickBallRRotation].X = tool->readvalue(fin, "[ContinuousKickBallRRotation].X", 0);
		USinfo->MoveValue[ContinuousKickBallRRotation].Y = tool->readvalue(fin, "[ContinuousKickBallRRotation].Y", 0);
		USinfo->MoveValue[ContinuousKickBallRRotation].theta = tool->readvalue(fin, "[ContinuousKickBallRRotation].theta", 0);
		
		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousLRotation].X = tool->readvalue(fin, "[ContinuousLRotation].X", 0);
		USinfo->MoveValue[ContinuousLRotation].Y = tool->readvalue(fin, "[ContinuousLRotation].Y", 0);
		USinfo->MoveValue[ContinuousLRotation].theta = tool->readvalue(fin, "[ContinuousLRotation].theta", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousRRotation].X = tool->readvalue(fin, "[ContinuousRRotation].X", 0);
		USinfo->MoveValue[ContinuousRRotation].Y = tool->readvalue(fin, "[ContinuousRRotation].Y", 0);
		USinfo->MoveValue[ContinuousRRotation].theta = tool->readvalue(fin, "[ContinuousRRotation].theta", 0);

		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousOneStep].X = tool->readvalue(fin, "[ContinuousOneStep].X", 0); 
		USinfo->MoveValue[ContinuousOneStep].Y = tool->readvalue(fin, "[ContinuousOneStep].Y", 0);
		USinfo->MoveValue[ContinuousOneStep].theta = tool->readvalue(fin, "[ContinuousOneStep].theta", 0);
		
		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousRStart].X = tool->readvalue(fin, "[ContinuousRStart].X", 0); 
		USinfo->MoveValue[ContinuousRStart].Y = tool->readvalue(fin, "[ContinuousRStart].Y", 0);
		USinfo->MoveValue[ContinuousRStart].theta = tool->readvalue(fin, "[ContinuousRStart].theta", 0);
		
		fin.getline(temp, sizeof(temp));
		USinfo->MoveValue[ContinuousLStart].X = tool->readvalue(fin, "[ContinuousLStart].X", 0); 
		USinfo->MoveValue[ContinuousLStart].Y = tool->readvalue(fin, "[ContinuousLStart].Y", 0);
		USinfo->MoveValue[ContinuousLStart].theta = tool->readvalue(fin, "[ContinuousLStart].theta", 0);

		fin.close();
		ROS_INFO("closefile BodyActionValue.ini");
	}
	catch (exception e)
	{
		ROS_INFO("catchfile BodyActionValue.ini");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "strategymain");
	ros::NodeHandle nh;
	KidsizeStrategy KidsizeStrategy(nh);

	ros::Rate loop_rate(30);

	KidsizeStrategy.initparameterpath();

	while (nh.ok()) 
	{
		KidsizeStrategy.strategymain();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void KidsizeStrategy::strategymain()
{
	if (strategy_info->getStrategyStart())
	{
		switch (USinfo->BodyStrategySelect)
		{
			case Initial:
				ROS_INFO("Initialization Function");
				USinfo->IMU_Yaw_initial=IMU_Yaw_Value();
				USinfo->IMU_Pitch_initial=IMU_Pitch_Value();
				Initialization();
				break;
			case Find_Soccer:
				ROS_INFO("FindSoccerBody Function");
				FindSoccerBody();
				break;
			case Trace_Soccer:
				ROS_INFO("TraceSoccerBody Function");
				TraceSoccerBody();
				USinfo->BodyStrategySelect = Find_Soccer;
				break;
		}
	}
	else
	{
		if (USinfo->ContinuousFlag)BodyAction(ContinuousStop);
		if (USinfo->ContinuousLRotationFlag)BodyAction(ContinuousStop);
		if (USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
		InitialMoveBodyFlag = false;
		USinfo->BodyStrategySelect = Initial;
	}
}

void KidsizeStrategy::Initialization()
{
	ReadIni_BodyActionValue();
	ReadIni_tracksoccer();
	USinfo->Initialize_tracksoccer();
	USinfo->MoveValue[Continuous].X = USinfo->MoveValue[ContinuousStart].X;
	USinfo->MoveValue[Continuous].Y = USinfo->MoveValue[ContinuousStart].Y;
	USinfo->MoveValue[Continuous].theta = USinfo->MoveValue[ContinuousStart].theta;
	USinfo->MoveValue[Continuous].Mode = USinfo->MoveValue[ContinuousStart].Mode;
	USinfo->MoveValue[Continuous].IMU = USinfo->MoveValue[ContinuousStart].IMU;
	ContinuousBigRotationTime = 0.0;
	ContinuousSmallRotationTime = 0.0;
	ContinuousRotationTime = 0.0 + USinfo->StartPositionError;
	BigRotationCnt = 0;
	SmallRotationCnt = 0;
	RotationCnt = 0;
	cannotsee = 0;
	IMU_SET = 0;
	ROS_INFO("29");
	ros_com->sendBodySector(29);//站立動作
	tool->Delay(2000);
	ROS_INFO("Sector 2");
	ros_com->sendBodySector(2);//站姿重心調整
	tool->Delay(1500);
	if(!InitialMoveBodyFlag)
	{
		InitialMoveBody();
	}
	USinfo->BodyStrategySelect = Find_Soccer;
}
float KidsizeStrategy::IMU_Yaw_Value()
{
	ros::spinOnce();
	return strategy_info->getIMUValue().Yaw - USinfo->IMU_Yaw_initial;
}
float KidsizeStrategy::IMU_Pitch_Value()
{
	ros::spinOnce();
	return strategy_info->getIMUValue().Pitch - USinfo->IMU_Pitch_initial;
}
void KidsizeStrategy::gettimeofday_tstart()
{
	ros::spinOnce();
	gettimeofday(&tstart, NULL);
}

void KidsizeStrategy::gettimeofday_tend()
{
	ros::spinOnce();
	gettimeofday(&tend, NULL);
	timeuse = (1000000*(tend.tv_sec - tstart.tv_sec) + (tend.tv_usec - tstart.tv_usec))/1000;
}

void KidsizeStrategy::InitialMoveBody()
{
	ROS_INFO("DIO = %d", strategy_info->DIOValue.DInput);
	if(strategy_info->DIOValue.Switch.D1 == 0)
	{
		if(strategy_info->DIOValue.Switch.D0 == 1 && strategy_info->DIOValue.Switch.D2 == 0)//11001		近左轉		100
		{
			ROS_INFO("DIOValue is 11001");
			BodyAction(Continuous, 4, USinfo->MoveValue[ContinuousOneStep].X, USinfo->MoveValue[ContinuousOneStep].Y, USinfo->MoveValue[ContinuousOneStep].theta, 25);
			gettimeofday_tstart();
			BodyAction(Continuous, 150, USinfo->MoveValue[ContinuousBigLRotation].X, USinfo->MoveValue[ContinuousBigLRotation].Y, USinfo->MoveValue[ContinuousBigLRotation].theta, 25);
			gettimeofday_tend();
			ContinuousBigRotationTime = ContinuousBigRotationTime + timeuse;
			IMU_SET = 90;
		}
		else if(strategy_info->DIOValue.Switch.D0 == 1 && strategy_info->DIOValue.Switch.D2 == 1)//11100	近右轉		101
		{
			ROS_INFO("DIOValue is 11100");
			BodyAction(Continuous, 1200, USinfo->MoveValue[ContinuousOneStep].X, USinfo->MoveValue[ContinuousOneStep].Y, USinfo->MoveValue[ContinuousOneStep].theta, 25);
			gettimeofday_tstart();
			BodyAction(Continuous, 400, USinfo->MoveValue[ContinuousBigRRotation].X, USinfo->MoveValue[ContinuousBigRRotation].Y, USinfo->MoveValue[ContinuousBigRRotation].theta, 25);
			gettimeofday_tend();
			ContinuousBigRotationTime = ContinuousBigRotationTime + timeuse;
			IMU_SET = -90;
		}
		else if(strategy_info->DIOValue.Switch.D0 == 0 && strategy_info->DIOValue.Switch.D2 == 1)//11011		左邊		001
		{
			IMU_SET = 90;
		}
		
	}
	else if(strategy_info->DIOValue.Switch.D1 == 1)
	{ 
		if(strategy_info->DIOValue.Switch.D0 == 0 && strategy_info->DIOValue.Switch.D2 == 0)//11011			遠右轉			010
		{
			ROS_INFO("DIOValue is 11011");
			BodyAction(Continuous, 800, USinfo->MoveValue[ContinuousOneStep].X, USinfo->MoveValue[ContinuousOneStep].Y, USinfo->MoveValue[ContinuousOneStep].theta, 25);
			gettimeofday_tstart();
			BodyAction(Continuous, 150, USinfo->MoveValue[ContinuousBigRRotation].X, USinfo->MoveValue[ContinuousBigRRotation].Y, USinfo->MoveValue[ContinuousBigRRotation].theta, 25);
			gettimeofday_tend();
			ContinuousBigRotationTime = ContinuousBigRotationTime - timeuse;
			BodyAction(Continuous, 50, USinfo->MoveValue[ContinuousOneStep].X, USinfo->MoveValue[ContinuousOneStep].Y, USinfo->MoveValue[ContinuousOneStep].theta, 25);
			IMU_SET = -90;
		}
		else if(strategy_info->DIOValue.Switch.D0 == 1 &&strategy_info->DIOValue.Switch.D2 == 0)//11110			遠左轉			110
		{
			ROS_INFO("DIOValue is 11101");
			BodyAction(Continuous, 800, USinfo->MoveValue[ContinuousOneStep].X, USinfo->MoveValue[ContinuousOneStep].Y, USinfo->MoveValue[ContinuousOneStep].theta, 25);
			gettimeofday_tstart();
			BodyAction(Continuous, 150, USinfo->MoveValue[ContinuousBigLRotation].X, USinfo->MoveValue[ContinuousBigLRotation].Y, USinfo->MoveValue[ContinuousBigLRotation].theta, 25);
			gettimeofday_tend();
			ContinuousBigRotationTime = ContinuousBigRotationTime - timeuse;
			BodyAction(Continuous, 50, USinfo->MoveValue[ContinuousOneStep].X, USinfo->MoveValue[ContinuousOneStep].Y, USinfo->MoveValue[ContinuousOneStep].theta, 25);
			IMU_SET = 90;
		}
		else if(strategy_info->DIOValue.Switch.D0 == 0 && strategy_info->DIOValue.Switch.D2 == 1)//11011		右轉		011
		{
			IMU_SET = -90;
		}
	}

	InitialMoveBodyFlag = true;

}

void KidsizeStrategy::FindSoccerBody()
{
	if(IMU_Pitch_Value()>45)
	{
		BodyAction(ContinuousStop);
		tool->Delay(5000);
		ros_com->sendBodySector(7);//後倒起身準備姿勢
		tool->Delay(1000);
		ros_com->sendBodySector(77);//後倒起身
		tool->Delay(1000);
		USinfo->BodyStrategySelect = Initial;
	}
	if ((USinfo->obstacle.YMin == 1 && USinfo->obstacle.YMax >= 60) || ((USinfo->obstacle.XMin == 1 || USinfo->obstacle.XMax == 318) && abs(USinfo->obstacle.YMax - USinfo->obstacle.YMin) >= 130) || USinfo->obstacle.YMax == 238)//120???
	{
		ROS_INFO("---------------------------------------\n\n");
		AvoidObstacle();
	}
	else if(USinfo->seesoccer)
	{
		ROS_INFO("---------------------------------------\n\n");
		USinfo->BodyStrategySelect = Trace_Soccer;
	}
	else
	{
		ROS_INFO("USinfo->seesoccer = %d", USinfo->seesoccer);
		BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousOneStep].X, USinfo->MoveValue[ContinuousOneStep].Y, USinfo->MoveValue[ContinuousOneStep].theta, 25);
		ROS_INFO("IMU== %f",IMU_Pitch_Value());
	}
}

void KidsizeStrategy::TraceSoccerBody()
{
	if(IMU_Pitch_Value()>45)
	{
		BodyAction(ContinuousStop);
		tool->Delay(5000);
		ros_com->sendBodySector(7);
		tool->Delay(1000);
		ros_com->sendBodySector(77);
		tool->Delay(1000);
		USinfo->BodyStrategySelect = Initial;
	}
	if(USinfo->seesoccer)
	{
		ROS_INFO("---------------------------------------\n\n");
		if (USinfo->HeadVertical >= 1600)
		{
			ROS_INFO("USinfo->HeadVertical >= 1600");
			if (USinfo->HeadHorizontal > USinfo->FixHead_LB_Limit)//2108
			{
				ROS_INFO("Head Over Left Limit, USinfo->HeadHorizontal > %d", USinfo->FixHead_LB_Limit);
				if (abs((float)(USinfo->HeadHorizontal - USinfo->FixHead_LB_Limit) / USinfo->MotorMoveOneDegree) > 12) //
				{
					ROS_INFO("Degree >  10,  Turn Turn Left");
					ROS_INFO("Degree >  10 ==== %d",abs((float)(USinfo->HeadHorizontal - USinfo->FixHead_LB_Limit) / USinfo->MotorMoveOneDegree));
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousLRotation].X, USinfo->MoveValue[ContinuousLRotation].Y, USinfo->MoveValue[ContinuousLRotation].theta, 25);
				}
				else
				{
					ROS_INFO("Degree <= 10, Big Turn Left");
					ROS_INFO("Degree <=  10 ==== %d", abs((float)(USinfo->HeadHorizontal - USinfo->FixHead_LB_Limit) / USinfo->MotorMoveOneDegree));
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousBigLRotation].X, USinfo->MoveValue[ContinuousBigLRotation].Y, USinfo->MoveValue[ContinuousBigLRotation].theta, 25);
				}
			}
			else if (USinfo->FixHead_RB_Limit > USinfo->HeadHorizontal)//1928
			{
				ROS_INFO("Head Over Right Limit, %d > USinfo->HeadHorizontal", USinfo->FixHead_RB_Limit);
				if (abs((float)(USinfo->FixHead_RB_Limit - USinfo->HeadHorizontal) / USinfo->MotorMoveOneDegree) > 12)
				{
					ROS_INFO("Degree >  10, Turn Turn Right");
					ROS_INFO("Degree >  10 ==== %d", abs((float)(USinfo->FixHead_RB_Limit - USinfo->HeadHorizontal) / USinfo->MotorMoveOneDegree));
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousRRotation].X, USinfo->MoveValue[ContinuousRRotation].Y, USinfo->MoveValue[ContinuousRRotation].theta, 25);
				}
				else
				{
					ROS_INFO("Degree <= 10, Big Turn Right");
					ROS_INFO("Degree >  10 ==== %d", abs((float)(USinfo->FixHead_RB_Limit - USinfo->HeadHorizontal) / USinfo->MotorMoveOneDegree));
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousBigRRotation].X, USinfo->MoveValue[ContinuousBigRRotation].Y, USinfo->MoveValue[ContinuousBigRRotation].theta, 25);
				}
			}
			else
			{
				ROS_INFO("Go Straight");
				BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousOneStep].X, USinfo->MoveValue[ContinuousOneStep].Y, USinfo->MoveValue[ContinuousOneStep].theta, 25);
			}
		}
		else if (1600 > USinfo->HeadVertical && USinfo->HeadVertical >=  USinfo->KickSoccerHeadVertical)//1270
		{
			ROS_INFO("1600 > USinfo->HeadVertical >= %d",  USinfo->KickSoccerHeadVertical);
			if (USinfo->HeadHorizontal > USinfo->FixHead_LB_Limit - USinfo->MiddleDecade)
			{
				ROS_INFO("Head Over Left Limit, USinfo->HeadHorizontal > %d", USinfo->FixHead_LB_Limit - USinfo->MiddleDecade);
				if (abs((float)(USinfo->HeadHorizontal - (USinfo->FixHead_LB_Limit - USinfo->MiddleDecade)) / USinfo->MotorMoveOneDegree) > 10)
				{
					ROS_INFO("Degree >  10, Turn Turn Left");
					ROS_INFO("Degree >  10 ==== %d", abs((float)(USinfo->HeadHorizontal - (USinfo->FixHead_LB_Limit - USinfo->MiddleDecade)) / USinfo->MotorMoveOneDegree));
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousLRotation].X, USinfo->MoveValue[ContinuousLRotation].Y, USinfo->MoveValue[ContinuousLRotation].theta, 25);
				}
				else
				{
					ROS_INFO("Degree <= 10, Small Turn Left");
					ROS_INFO("Degree >  10 ==== %d", abs((float)(USinfo->HeadHorizontal - (USinfo->FixHead_LB_Limit - USinfo->MiddleDecade)) / USinfo->MotorMoveOneDegree));
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousSmallLRotation].X, USinfo->MoveValue[ContinuousSmallLRotation].Y, USinfo->MoveValue[ContinuousSmallLRotation].theta, 25);
				}
			}
			else if (USinfo->FixHead_RB_Limit - USinfo->MiddleDecade > USinfo->HeadHorizontal)
			{
				ROS_INFO("Head Over Right Limit, %d > USinfo->HeadHorizontal", USinfo->FixHead_RB_Limit - USinfo->MiddleDecade);
				if (abs((float)(USinfo->FixHead_RB_Limit - USinfo->MiddleDecade - USinfo->HeadHorizontal) / USinfo->MotorMoveOneDegree) > 10)
				{
					ROS_INFO("Degree >  10, Trun Turn Right");
					ROS_INFO("Degree >  10 ==== %d", abs((float)(USinfo->FixHead_RB_Limit - USinfo->MiddleDecade - USinfo->HeadHorizontal) / USinfo->MotorMoveOneDegree));
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousRRotation].X, USinfo->MoveValue[ContinuousRRotation].Y, USinfo->MoveValue[ContinuousRRotation].theta, 25);
				}
				else
				{
					ROS_INFO("Degree <= 10, Small Turn Right");
					ROS_INFO("Degree >  10 ==== %d", abs((float)(USinfo->FixHead_RB_Limit - USinfo->MiddleDecade - USinfo->HeadHorizontal) / USinfo->MotorMoveOneDegree));
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousSmallRRotation].X, USinfo->MoveValue[ContinuousSmallRRotation].Y, USinfo->MoveValue[ContinuousSmallRRotation].theta, 25);
				}
			}
			else
			{
				ROS_INFO("Go Straight");
				BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousOneStep].X-300, USinfo->MoveValue[ContinuousOneStep].Y, USinfo->MoveValue[ContinuousOneStep].theta, 25);
			}
			
		}
		else if (USinfo->KickSoccerHeadVertical > USinfo->HeadVertical)//1270
		{
			ROS_INFO("%d > USinfo->HeadVertical",  USinfo->KickSoccerHeadVertical);
			
			ROS_INFO("USinfo->HeadVertical:%d", USinfo->HeadVertical);
			ROS_INFO("USinfo->HeadHorizontal:%d", USinfo->HeadHorizontal);
			if (USinfo->HeadHorizontal > USinfo->FixHead_LB_Limit - USinfo->NearDecade && USinfo->HeadVertical >= USinfo->KickSoccerHeadVertical)
			{
				ROS_INFO("USinfo->HeadVertical > %d", USinfo->KickSoccerHeadVertical);
				ROS_INFO("Head Over Left Limit, USinfo->HeadHorizontal > %d", USinfo->FixHead_LB_Limit - USinfo->NearDecade);
				if (abs((float)(USinfo->HeadHorizontal - USinfo->NearDecade - USinfo->FixHead_LB_Limit) / USinfo->MotorMoveOneDegree) > 3)
				{
					ROS_INFO("Degree >  10, Big Turn Left");
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousRRotation].X, USinfo->MoveValue[ContinuousRRotation].Y, USinfo->MoveValue[ContinuousRRotation].theta, 25);
				}
				else
				{
					ROS_INFO("Degree <= 10, Small Turn Left");
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousRRotation].X, USinfo->MoveValue[ContinuousRRotation].Y, USinfo->MoveValue[ContinuousRRotation].theta, 25);
				}
			}
			else if (USinfo->HeadHorizontal < USinfo->FixHead_RB_Limit - USinfo->NearDecade && USinfo->HeadVertical >= USinfo->KickSoccerHeadVertical )
			{
				ROS_INFO("USinfo->HeadVertical > %d", USinfo->KickSoccerHeadVertical);
				ROS_INFO("Head Over Right Limit, %d > USinfo->HeadHorizontal", USinfo->FixHead_RB_Limit - USinfo->NearDecade);
				if (abs((float)(USinfo->FixHead_RB_Limit - USinfo->NearDecade - USinfo->HeadHorizontal) / USinfo->MotorMoveOneDegree) > 3)
				{
					ROS_INFO("Degree >  10, Big Turn Right");
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousLRotation].X, USinfo->MoveValue[ContinuousLRotation].Y, USinfo->MoveValue[ContinuousLRotation].theta, 25);
				}
				else
				{
					ROS_INFO("Degree <= 10, Small Turn Right");
					BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousLRotation].X, USinfo->MoveValue[ContinuousLRotation].Y, USinfo->MoveValue[ContinuousLRotation].theta, 25);
				}
			}
			else if( USinfo->soccer.size > 2000 && USinfo->KickSoccerHeadVertical > USinfo->HeadVertical && 2000< USinfo->HeadHorizontal <2100)
			{
				ROS_INFO("%d > USinfo->HeadVertical", USinfo->KickSoccerHeadVertical);
				ROS_INFO("Just KickSoccer");

				ROS_INFO("USinfo->AimGoalFlag: %d", USinfo->AimGoalFlag);
				BodyAction(Continuous, 20, USinfo->MoveValue[ContinuousStart].X, USinfo->MoveValue[ContinuousStart].Y, USinfo->MoveValue[ContinuousStart].theta, 25);

				if (USinfo->AimGoalFlag)
				{
					AimGoal();
				}
				if(!IMU_Ready){
					TraceSoccerBody();
				}
				
				while(!((USinfo->HeadHorizontal <1920 && USinfo->HeadHorizontal >1630)||(USinfo->HeadHorizontal <2450 && USinfo->HeadHorizontal >2250)))
				{
					while (USinfo->HeadHorizontal <= 2048 && USinfo->HeadHorizontal >= 1920 || USinfo->HeadHorizontal >= 2450)
					{
						ROS_INFO("USinfo->HeadHorizontal == %d",USinfo->HeadHorizontal);
						ros::spinOnce();
						BodyAction(Continuous, 20, USinfo->MoveValue[ContinuousSmallLMove].X, USinfo->MoveValue[ContinuousSmallLMove].Y, USinfo->MoveValue[ContinuousSmallLMove].theta, 25);
						if(!( USinfo->soccer.size > 2500 ))
						{
							TraceSoccerBody();
						}
	
					}

					while (USinfo->HeadHorizontal >= 2048 && USinfo->HeadHorizontal <= 2250 || USinfo->HeadHorizontal <= 1630)
					{
						ROS_INFO("USinfo->HeadHorizontal == %d",USinfo->HeadHorizontal);
						ros::spinOnce();
						BodyAction(Continuous, 20, USinfo->MoveValue[ContinuousSmallLMove].X, USinfo->MoveValue[ContinuousSmallLMove].Y, USinfo->MoveValue[ContinuousSmallLMove].theta, 25);
						if(!( USinfo->soccer.size > 2500 ))
						{
							TraceSoccerBody();
						}
	
					}
				}
				BodyAction(Continuous, 20, USinfo->MoveValue[ContinuousStart].X, USinfo->MoveValue[ContinuousStart].Y, USinfo->MoveValue[ContinuousStart].theta, 25);
				BodyAction(ContinuousStop);
				ros::spinOnce();

				ROS_INFO("USinfo->soccer.Y: %d", USinfo->soccer.Y);

				if (120 - USinfo->soccer.Y < 30)
				{

					ROS_INFO("29");
					ros_com->sendBodySector(29);
					tool->Delay(1500);

					if (USinfo->HeadHorizontal <= 2048 )
					{
						ROS_INFO("USinfo->HeadHorizontal <= 2048");
						ROS_INFO("Right Foot");
						ROS_INFO("Sector 1");
						ros_com->sendBodySector(1);//右腳梯球
						tool->Delay(9500);
					}
					else if(USinfo->HeadHorizontal > 2048)
					{
						ROS_INFO("USinfo->HeadHorizontal > 2048");
						ROS_INFO("Leftt Foot");
						ROS_INFO("Sector 3");
						ros_com->sendBodySector(3);//左腳踢球
						tool->Delay(9500);
					}

					USinfo->BodyStrategySelect = Initial;
				}
				else
				{
					USinfo->BodyStrategySelect = Find_Soccer;
				}
			}
			else
			{
				ROS_INFO("1170 > USinfo->HeadVertical >= %d", USinfo->KickSoccerHeadVertical);
				ROS_INFO("Go Straight");
				BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousStart].X, USinfo->MoveValue[ContinuousStart].Y, USinfo->MoveValue[ContinuousStart].theta, 25);
			}
		}
		else
		{
			ROS_INFO("Error");
			BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousStart].X, USinfo->MoveValue[ContinuousStart].Y, USinfo->MoveValue[ContinuousStart].theta, 25);
		}
	}
	else
	{		
		USinfo->BodyStrategySelect = Find_Soccer;
	}
}

void KidsizeStrategy::AvoidObstacle()
{
	ROS_INFO("----------------------------------------");
	if(IMU_Pitch_Value()>45)
		{
			BodyAction(ContinuousStop);
			tool->Delay(5000);
			ros_com->sendBodySector(7);
			tool->Delay(1000);
			ros_com->sendBodySector(77);
			tool->Delay(1000);
			USinfo->BodyStrategySelect = Initial;
		}
	if (USinfo->seesoccer)
	{
		
		ROS_INFO("Both See Obstacle & Soccer");
		if(USinfo->soccer.Y < USinfo->obstacle.Y || USinfo->obstacle.YMax == 238) //球比障礙物遠   障礙物超級近
		{
			ROS_INFO("Too far or close,just only AvoidObstacle");
			USinfo->seesoccer = false;
			AvoidObstacle();
		}
		else if(1180 < USinfo->HeadVertical &&  USinfo->obstacle.YMax < 190)   //
		{
			ROS_INFO("Too far ,just only TraceSoccer");
			TraceSoccerBody();
		}
		else if(USinfo->HeadHorizontal <2000)
		{
			BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousRRotation].X, USinfo->MoveValue[ContinuousRRotation].Y, USinfo->MoveValue[ContinuousRRotation].theta, 25);
		}
		else if(USinfo->HeadHorizontal >2100)
		{
			BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousLRotation].X, USinfo->MoveValue[ContinuousLRotation].Y, USinfo->MoveValue[ContinuousLRotation].theta, 25);
		}
		else if(USinfo->soccer.X < USinfo->obstacle.XMin && 2100>USinfo->HeadHorizontal >2000)   //球在障礙物左邊
		{
			if(USinfo->soccer.X <= 15 && (USinfo->obstacle.XMax - USinfo->obstacle.XMin) < 20 && USinfo->obstacle.XMax == 318)		//球在最左邊  障礙物在很右邊   左平移
			{
				ROS_INFO("LeftMove && BigTurnRight");
			    BodyAction(Continuous, 10, USinfo->MoveValue[ContinuousLMove].X, USinfo->MoveValue[ContinuousLMove].Y, USinfo->MoveValue[ContinuousLMove].theta, 50);
			}
			else																													//右旋
			{
				ROS_INFO("OBS BigTurnRight");
				BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousBigRRotation].X, USinfo->MoveValue[ContinuousBigRRotation].Y, USinfo->MoveValue[ContinuousBigRRotation].theta, 25);
			}
		}
		else if(USinfo->soccer.X > USinfo->obstacle.XMax && 2100>USinfo->HeadHorizontal >2000)	//球在障礙物右邊
		{
			if(USinfo->soccer.X >= (318 -15) && (USinfo->obstacle.XMax - USinfo->obstacle.XMin) < 20 && USinfo->obstacle.XMax == 1)			//球在最右邊 障礙物在最左邊
			{
				ROS_INFO("RightMove && BigTurnLeft");
			    BodyAction(Continuous, 5, USinfo->MoveValue[ContinuousRMove].X, USinfo->MoveValue[ContinuousRMove].Y, USinfo->MoveValue[ContinuousRMove].theta, 50);
			}
			else
			{
				ROS_INFO("OBS BigTurnLeft");
				BodyAction(Continuous, 2, USinfo->MoveValue[ContinuousBigLRotation].X, USinfo->MoveValue[ContinuousBigLRotation].Y, USinfo->MoveValue[ContinuousBigLRotation].theta, 25);
			}
		}
		else if(USinfo->obstacle.XMin <= USinfo->soccer.X && USinfo->soccer.X <= USinfo->obstacle.XMax	&& 2100>USinfo->HeadHorizontal >2000)				//球跟障礙物
		{
			ROS_INFO("Soccer to Aim Obstacle");
			if(USinfo->soccer.X < (USinfo->obstacle.X - (USinfo->obstacle.X - USinfo->obstacle.XMin)*0.2) && !(USinfo->obstacle.XMin <= 20 && USinfo->obstacle.XMax >= 300))   //球偏左  
			{
				ROS_INFO("SmallTurnLeft");
				BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousSmallLRotation].X, USinfo->MoveValue[ContinuousSmallLRotation].Y, USinfo->MoveValue[ContinuousSmallLRotation].theta, 25);
			}
			else if(USinfo->soccer.X > (USinfo->obstacle.X + (USinfo->obstacle.XMax - USinfo->obstacle.X)*0.2) && !(USinfo->obstacle.XMin <= 20 && USinfo->obstacle.XMax >= 300))		//球偏右
			{
				ROS_INFO("SmallTurnright");
				BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousSmallRRotation].X, USinfo->MoveValue[ContinuousSmallRRotation].Y, USinfo->MoveValue[ContinuousSmallRRotation].theta, 25);
			}
			else
			{
				TraceSoccerBody();
			}
		}
	}
	else
	{
		ROS_INFO("AvoidObstacle");
		ROS_INFO("USinfo->obstacle.XMin: %d", USinfo->obstacle.XMin);
		ROS_INFO("USinfo->obstacle.XMax: %d", USinfo->obstacle.XMax);
		if (USinfo->obstacle.XMin == 1 && USinfo->obstacle.XMax == 318)		//有點太近
		{
			ROS_INFO("BigTurnLeft");
			BodyAction(Continuous, 4, USinfo->MoveValue[ContinuousRRotation].X, USinfo->MoveValue[ContinuousRRotation].Y, USinfo->MoveValue[ContinuousRRotation].theta, 25);
		}
		else if(1500 < USinfo->HeadHorizontal && USinfo->HeadHorizontal < 1900)//頭在右邊
		{
			ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
			ROS_INFO("%d",USinfo->HeadHorizontal);
			BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousLMove].X, USinfo->MoveValue[ContinuousLMove].Y, USinfo->MoveValue[ContinuousLMove].theta, 25);
		}
		else if(2600 > USinfo->HeadHorizontal && USinfo->HeadHorizontal > 2200)//頭在左邊
		{
			ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
			ROS_INFO("%d",USinfo->HeadHorizontal);
			BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousRMove].X, USinfo->MoveValue[ContinuousRMove].Y, USinfo->MoveValue[ContinuousRMove].theta, 25);
		}
        else if (USinfo->obstacle.XMin == 1 && USinfo->obstacle.XMax < 110 && 2200>USinfo->HeadHorizontal >1900)		//左三分之一		障礙物在左前
		{
				ROS_INFO("Too close BigTurnLeft & RightMove");
				BodyAction(Continuous, 2, USinfo->MoveValue[ContinuousBigLRotation].X, USinfo->MoveValue[ContinuousBigLRotation].Y, USinfo->MoveValue[ContinuousBigLRotation].theta, 25);
				BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousRMove].X, USinfo->MoveValue[ContinuousRMove].Y, USinfo->MoveValue[ContinuousRMove].theta, 25);
		}
		else if (200 < USinfo->obstacle.XMin && USinfo->obstacle.XMax == 318 && 2200>USinfo->HeadHorizontal >1900) 	//右三分之一		障礙物在右前
		{
				ROS_INFO("Too close BigTurnRight & LeftMove");
				BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousBigRRotation].X, USinfo->MoveValue[ContinuousBigRRotation].Y, USinfo->MoveValue[ContinuousBigRRotation].theta, 25);//1000
				BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousLMove].X, USinfo->MoveValue[ContinuousLMove].Y, USinfo->MoveValue[ContinuousLMove].theta, 25);
		}
		else if (USinfo->obstacle.XMin == 1 && USinfo->obstacle.XMax < 318  && 2200>USinfo->HeadHorizontal >1900)		
		{
			ROS_INFO("Right RightMove");
			BodyAction(Continuous, 4, USinfo->MoveValue[ContinuousRMove].X, USinfo->MoveValue[ContinuousRMove].Y, USinfo->MoveValue[ContinuousRMove].theta, 25);
		}
		else if (USinfo->obstacle.XMin > 1 && USinfo->obstacle.XMax == 318  && 2200>USinfo->HeadHorizontal >1900)
		{
			ROS_INFO("Left LeftMove");
			BodyAction(Continuous, 4, USinfo->MoveValue[ContinuousLMove].X, USinfo->MoveValue[ContinuousLMove].Y, USinfo->MoveValue[ContinuousLMove].theta, 25);
		}
		else if (USinfo->obstacle.XMin > 1 && USinfo->obstacle.XMax < 318   && 2200>USinfo->HeadHorizontal >1900)
		{
			if ((USinfo->obstacle.XMin -1) <= (318 - USinfo->obstacle.XMax))
			{
				ROS_INFO("Middle RightMove");
				BodyAction(Continuous, 4, USinfo->MoveValue[ContinuousRMove].X, USinfo->MoveValue[ContinuousRMove].Y, USinfo->MoveValue[ContinuousRMove].theta, 25);
			}
			else if ((USinfo->obstacle.XMin - 1) > (318 - USinfo->obstacle.XMax))
			{
				ROS_INFO("Middle LeftMove");
				BodyAction(Continuous, 4, USinfo->MoveValue[ContinuousLMove].X, USinfo->MoveValue[ContinuousLMove].Y, USinfo->MoveValue[ContinuousLMove].theta, 25);
			}
		}
	}
}

void KidsizeStrategy::AimGoal()
{
	ROS_INFO("AimGoal");
	
	while(IMU_Yaw_Value()> (0+IMU_SET) && USinfo->soccer.size > 2000)
	{
	ROS_INFO("IMU_Yaw_Value = %f",IMU_Yaw_Value());
	BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousKickBallRRotation].X, USinfo->MoveValue[ContinuousKickBallRRotation].Y, USinfo->MoveValue[ContinuousKickBallRRotation].theta, 25);
	
		if(IMU_Pitch_Value()>45)
		{
			BodyAction(ContinuousStop);
			tool->Delay(5000);
			ros_com->sendBodySector(7);
			tool->Delay(1000);
			ros_com->sendBodySector(77);
			tool->Delay(1000);
			USinfo->BodyStrategySelect = Initial;
		}
	}
	

	
	while(IMU_Yaw_Value()<= (0+IMU_SET) && USinfo->soccer.size > 2000 )
	{
	ROS_INFO("IMU_Yaw_Value = %f",IMU_Yaw_Value());
	BodyAction(Continuous, 1, USinfo->MoveValue[ContinuousKickBallLRotation].X, USinfo->MoveValue[ContinuousKickBallLRotation].Y, USinfo->MoveValue[ContinuousKickBallLRotation].theta, 25);
		if(IMU_Pitch_Value()>45)
		{
			BodyAction(ContinuousStop);
			tool->Delay(5000);
			ros_com->sendBodySector(7);
			tool->Delay(1000);
			ros_com->sendBodySector(77);
			tool->Delay(1000);
			USinfo->BodyStrategySelect = Initial;
		}
	}

	if(!( USinfo->soccer.size > 2300)){
		TraceSoccerBody();
	}
	IMU_Ready = true;
}

void KidsizeStrategy::BodyAction(BodyActionState BodyActionSelect, int count, int MaxValueX, int MaxValueY, int MaxValueTheta, int DelayTime)
{
	for (int i = 0; i < count; i++)
	{
		switch (BodyActionSelect)
		{
		case SmallMove:
			ROS_INFO("SmallMove");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[SmallMove].X, USinfo->MoveValue[SmallMove].Y, 0, USinfo->MoveValue[SmallMove].theta, (WalkingMode)USinfo->MoveValue[SmallMove].Mode, (SensorMode)USinfo->MoveValue[SmallMove].IMU);
			tool->Delay(3000);
			break;
		case Move:
			ROS_INFO("Move");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[Move].X, USinfo->MoveValue[Move].Y, 0, USinfo->MoveValue[Move].theta, (WalkingMode)USinfo->MoveValue[Move].Mode, (SensorMode)USinfo->MoveValue[Move].IMU);
			tool->Delay(3000);
			break;
		case BigMove:
			ROS_INFO("BigMove");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[BigMove].X, USinfo->MoveValue[BigMove].Y, 0, USinfo->MoveValue[BigMove].theta, (WalkingMode)USinfo->MoveValue[BigMove].Mode, (SensorMode)USinfo->MoveValue[BigMove].IMU);
			tool->Delay(3000);
			break;
		case SmallLMove:
			ROS_INFO("SmallLMove");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[SmallLMove].X, USinfo->MoveValue[SmallLMove].Y, 0, USinfo->MoveValue[SmallLMove].theta, (WalkingMode)USinfo->MoveValue[SmallLMove].Mode, (SensorMode)USinfo->MoveValue[SmallLMove].IMU);
			tool->Delay(3000);
			break;
		case LMove:
			ROS_INFO("LMove");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[LMove].X, USinfo->MoveValue[LMove].Y, 0, USinfo->MoveValue[LMove].theta, (WalkingMode)USinfo->MoveValue[LMove].Mode, (SensorMode)USinfo->MoveValue[LMove].IMU);
			tool->Delay(3000);
			break;
		case SmallRMove:
			ROS_INFO("SmallRMove");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[SmallRMove].X, USinfo->MoveValue[SmallRMove].Y, 0, USinfo->MoveValue[SmallRMove].theta, (WalkingMode)USinfo->MoveValue[SmallRMove].Mode, (SensorMode)USinfo->MoveValue[SmallRMove].IMU);
			tool->Delay(3000);
			break;
		case RMove:
			ROS_INFO("RMove");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[RMove].X, USinfo->MoveValue[RMove].Y, 0, USinfo->MoveValue[RMove].theta, (WalkingMode)USinfo->MoveValue[RMove].Mode, (SensorMode)USinfo->MoveValue[RMove].IMU);
			tool->Delay(3000);
			break;
		case SmallLRotation:
			ROS_INFO("SmallLRotation");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[SmallLRotation].X, USinfo->MoveValue[SmallLRotation].Y, 0, USinfo->MoveValue[SmallLRotation].theta, (WalkingMode)USinfo->MoveValue[SmallLRotation].Mode, (SensorMode)USinfo->MoveValue[SmallLRotation].IMU);
			tool->Delay(3000);
			break;
		case LRotation:
			ROS_INFO("LRotation");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[LRotation].X, USinfo->MoveValue[LRotation].Y, 0, USinfo->MoveValue[LRotation].theta, (WalkingMode)USinfo->MoveValue[LRotation].Mode, (SensorMode)USinfo->MoveValue[LRotation].IMU);
			tool->Delay(3000);
			break;
		case BigLRotation:
			ROS_INFO("BigLRotation");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[BigLRotation].X, USinfo->MoveValue[BigLRotation].Y, 0, USinfo->MoveValue[BigLRotation].theta, (WalkingMode)USinfo->MoveValue[BigLRotation].Mode, (SensorMode)USinfo->MoveValue[BigLRotation].IMU);
			tool->Delay(3000);
			break;
		case SmallRRotation:
			ROS_INFO("SmallRRotation");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[SmallRRotation].X, USinfo->MoveValue[SmallRRotation].Y, 0, USinfo->MoveValue[SmallRRotation].theta, (WalkingMode)USinfo->MoveValue[SmallRRotation].Mode, (SensorMode)USinfo->MoveValue[SmallRRotation].IMU);
			tool->Delay(3000);
			break;
		case RRotation:
			ROS_INFO("RRotation");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[RRotation].X, USinfo->MoveValue[RRotation].Y, 0, USinfo->MoveValue[RRotation].theta, (WalkingMode)USinfo->MoveValue[RRotation].Mode, (SensorMode)USinfo->MoveValue[RRotation].IMU);
			tool->Delay(3000);
			break;
		case BigRRotation:
			ROS_INFO("BigRRotation");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[BigRRotation].X, USinfo->MoveValue[BigRRotation].Y, 0, USinfo->MoveValue[BigRRotation].theta, (WalkingMode)USinfo->MoveValue[BigRRotation].Mode, (SensorMode)USinfo->MoveValue[BigRRotation].IMU);
			tool->Delay(3000);
			break;
		case OneStep:
			ROS_INFO("OneStep");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[OneStep].X, USinfo->MoveValue[OneStep].Y, 0, USinfo->MoveValue[OneStep].theta, (WalkingMode)USinfo->MoveValue[OneStep].Mode, (SensorMode)USinfo->MoveValue[OneStep].IMU);
			tool->Delay(3000);
			break;
		case Back:
			ROS_INFO("Back");
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag)BodyAction(ContinuousStop);
			ros_com->sendBodyAuto(USinfo->MoveValue[Back].X, USinfo->MoveValue[Back].Y, 0, USinfo->MoveValue[Back].theta, (WalkingMode)USinfo->MoveValue[Back].Mode, (SensorMode)USinfo->MoveValue[Back].IMU);
			tool->Delay(3000);
			break;
		case ContinuousStart:
			if (!USinfo->ContinuousFlag) {
				ROS_INFO("ContinuousStart");
				USinfo->ContinuousFlag = true;
				ros_com->sendBodyAuto(USinfo->MoveValue[ContinuousStart].X, USinfo->MoveValue[ContinuousStart].Y, 0, USinfo->MoveValue[ContinuousStart].theta, (WalkingMode)USinfo->MoveValue[ContinuousStart].Mode, (SensorMode)USinfo->MoveValue[ContinuousStart].IMU);
				//tool->Delay(3000);
			}
			break;
		case ContinuousStop:
			if (USinfo->ContinuousFlag || USinfo->ContinuousLRotationFlag || USinfo->ContinuousRRotationFlag) {
				ROS_INFO("ContinuousStop");
				USinfo->ContinuousFlag = false;
				USinfo->ContinuousLRotationFlag = false;
				USinfo->ContinuousRRotationFlag = false;
				USinfo->MoveValue[ContinuousStart].X = 0;
				USinfo->MoveValue[ContinuousStart].Y = 0;
				USinfo->MoveValue[ContinuousStart].theta = 0;
				ros_com->sendBodyAuto(0, 0, 0, 0, (WalkingMode)USinfo->MoveValue[ContinuousStart].Mode, (SensorMode)USinfo->MoveValue[ContinuousStart].IMU);
				tool->Delay(3000);
			}
			break;
		case Continuous:
			ROS_INFO("Continuous");
			BodyAction(ContinuousStart);
			if (USinfo->ContinuousFlag) {
				USinfo->MoveValue[ContinuousShift].X > 0 ? ContinuousValseAdd(ShiftX, USinfo->MoveValue[ContinuousShift].X, MaxValueX) : ContinuousValseAdd(ShiftX, -USinfo->MoveValue[ContinuousShift].X, MaxValueX);
				USinfo->MoveValue[ContinuousShift].Y > 0 ? ContinuousValseAdd(ShiftY, USinfo->MoveValue[ContinuousShift].Y, MaxValueY) : ContinuousValseAdd(ShiftY, -USinfo->MoveValue[ContinuousShift].Y, MaxValueY);
				USinfo->MoveValue[ContinuousShift].theta > 0 ? ContinuousValseAdd(ShiftTheta, USinfo->MoveValue[ContinuousShift].theta, MaxValueTheta) : ContinuousValseAdd(ShiftTheta, -USinfo->MoveValue[ContinuousShift].theta, MaxValueTheta);
				ros_com->sendContinuousValue(USinfo->MoveValue[Continuous].X, USinfo->MoveValue[Continuous].Y, 0, USinfo->MoveValue[Continuous].theta, (SensorMode)USinfo->MoveValue[ContinuousStart].IMU);
			}
			tool->Delay(DelayTime);
			break;
		}
	}
}

void KidsizeStrategy::ContinuousValseAdd(ContinuousValseStates ContinuousValseState, int Value, int MaxValue)
{
	switch (ContinuousValseState)
	{
	case ShiftX:
		if (USinfo->MoveValue[Continuous].X > MaxValue) {
			USinfo->MoveValue[Continuous].X -= Value;
		}
		else if (USinfo->MoveValue[Continuous].X < MaxValue) {
			USinfo->MoveValue[Continuous].X += Value;
		}
		break;
	case ShiftY:
		if (USinfo->MoveValue[Continuous].Y > MaxValue) {
			USinfo->MoveValue[Continuous].Y -= Value;
		}
		else if (USinfo->MoveValue[Continuous].Y < MaxValue) {
			USinfo->MoveValue[Continuous].Y += Value;
		}
		break;
	case ShiftTheta:
		if (USinfo->MoveValue[Continuous].theta > MaxValue) {
			USinfo->MoveValue[Continuous].theta -= Value;
		}
		else if (USinfo->MoveValue[Continuous].theta < MaxValue) {
			USinfo->MoveValue[Continuous].theta += Value;
		}
		break;
	}
}

void KidsizeStrategy::PrintTerminal()
{
	printf("\n");
	ROS_INFO("Continuous.X = %d", USinfo->MoveValue[Continuous].X);
	ROS_INFO("Continuous.theta = %d", USinfo->MoveValue[Continuous].theta);
	ROS_INFO("USinfo->soccer.X = %d", USinfo->soccer.X);
	ROS_INFO("USinfo->soccer.XMin = %d", USinfo->soccer.XMin);
	ROS_INFO("USinfo->soccer.XMax = %d", USinfo->soccer.XMax);
	ROS_INFO("________________________________________");
}
