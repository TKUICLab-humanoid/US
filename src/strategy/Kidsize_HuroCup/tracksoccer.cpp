#include "strategy/tracksoccer.h"

void KidsizeImage::SendObstacleData()
{
	strategy::ObstacleData msg;
	msg.obstacleX = USinfo->obstacle.X;
	msg.obstacleY = USinfo->obstacle.Y;
	msg.obstacleXMin = USinfo->obstacle.XMin;
	msg.obstacleXMax = USinfo->obstacle.XMax;
	msg.obstacleYMin = USinfo->obstacle.YMin;
	msg.obstacleYMax = USinfo->obstacle.YMax;
	ObstacleData_Publish.publish(msg);
}

void KidsizeImage::SendSoccerData()
{
	strategy::SoccerData msg;
	msg.soccerX = USinfo->soccer.X;
	msg.soccerY = USinfo->soccer.Y;
	msg.soccerXMin = USinfo->soccer.XMin;
	msg.soccerXMax = USinfo->soccer.XMax;
	msg.soccerYMin = USinfo->soccer.YMin;
	msg.soccerYMax = USinfo->soccer.YMax;
	msg.soccersize = USinfo->soccer.size;
	msg.seesoccer = USinfo->seesoccer;
	SoccerData_Publish.publish(msg);
}

void KidsizeImage::SendHeadHV()
{
	strategy::HeadHV msg;
	msg.HeadHorizontal = USinfo->HorizontalHeadPosition;
	msg.HeadVertical = USinfo->VerticalHeadPosition;
	HeadHV_Publish.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image");
	ros::NodeHandle nh;
	KidsizeImage KidsizeImage(nh);

	USinfo->Initialize_tracksoccer();

	ros::Rate loop_rate(30);

	while (nh.ok()) 
	{
		ros::spinOnce();
		KidsizeImage.Imagemain();
		KidsizeImage.DrawFunction();
		loop_rate.sleep();
		//KidsizeImage.PrintTerminal();
	}
	return 0;
}

void KidsizeImage::Imagemain()
{
	while(IMU_Pitch_Value()>40)
	{
		MoveHead(HeadMotorID::VerticalID, 2048 ,500); 
	}
	if (strategy_info->getStrategyStart())
	{
		strategyclassify();				//影像判斷
		if(USinfo->avoidobsflag)
		{
			MoveHead(HeadMotorID::HorizontalID, USinfo->HorizontalHeadPosition, 200);
			MoveHead(HeadMotorID::VerticalID,  USinfo->VerticalHeadPosition, 200);
		}
		else if (USinfo->seesoccer)
		{
			TraceSoccerHead();			//對準球之類的
		}
		else
		{	
			FindsoccerHead();			//頭找球
		}	
	}
	else 
	{
		USinfo->Initialize_tracksoccer();			//應該是關指撥歸0八
		MoveHead(HeadMotorID::HorizontalID, USinfo->HeadHorizontalIni, 200);
		MoveHead(HeadMotorID::VerticalID, USinfo->HeadVerticalIni, 200);
	}
	SendHeadHV();			//存頭的馬達位子之類
}
float KidsizeImage::IMU_Pitch_Value()
{
	ros::spinOnce();
	return strategy_info->getIMUValue().Pitch - USinfo->IMU_Pitch_initial;
}
void KidsizeImage::DrawFunction()
{
	ros_com->drawImageFunction(1, DrawMode::DrawObject, 190, 250, 130, 190, 128, 0, 128);
	ros_com->drawImageFunction(2, DrawMode::DrawObject, USinfo->soccer.XMin, USinfo->soccer.XMax, USinfo->soccer.YMin, USinfo->soccer.YMax, 255, 0, 0);
	ros_com->drawImageFunction(3, DrawMode::DrawObject, USinfo->obstacle.XMin, USinfo->obstacle.XMax, USinfo->obstacle.YMin, USinfo->obstacle.YMax, 0, 255, 0);
	ros_com->drawImageFunction(4, DrawMode::DrawObject, 0, 320, USinfo->SoccerHortionalBaseLine, USinfo->SoccerHortionalBaseLine, 0, 0, 255);
	ros_com->drawImageFunction(5, DrawMode::DrawObject, USinfo->SoccerVerticalBaseLine ,USinfo->SoccerVerticalBaseLine , 0,240, 0, 0, 255);
	ros_com->drawImageFunction(6, DrawMode::DrawObject, USinfo->SoccerVerticalBaseLine - USinfo->VerticalRange , USinfo->SoccerVerticalBaseLine + USinfo->VerticalRange, \
		USinfo->SoccerHortionalBaseLine - USinfo->HorizontalRange,USinfo->SoccerHortionalBaseLine + USinfo->HorizontalRange, 0, 0, 255);
}

void KidsizeImage::strategyclassify()
{
	strategy_info->get_image_flag = true;
	ros::spinOnce();
	//---------------Obstacle------------------
	USinfo->Initialize_Obstacle();
	if (strategy_info->color_mask_subject_cnts[(int)LabelModel::Blue] == 0)
	{
		USinfo->seeobsflag = false;
	}
	for (int i = 0; i < strategy_info->color_mask_subject_cnts[(int)LabelModel::Blue]; ++i)
	{
		if (strategy_info->color_mask_subject[(int)LabelModel::Blue][i].size > 5000)//find max target
		{
			USinfo->obstacle.X = strategy_info->color_mask_subject[(int)LabelModel::Blue][i].X;
			USinfo->obstacle.Y = strategy_info->color_mask_subject[(int)LabelModel::Blue][i].Y;
			USinfo->obstacle.XMin = strategy_info->color_mask_subject[(int)LabelModel::Blue][i].XMin;
			USinfo->obstacle.YMin = strategy_info->color_mask_subject[(int)LabelModel::Blue][i].YMin;
			USinfo->obstacle.XMax = strategy_info->color_mask_subject[(int)LabelModel::Blue][i].XMax;
			USinfo->obstacle.YMax = strategy_info->color_mask_subject[(int)LabelModel::Blue][i].YMax;
			USinfo->obstacle.Width = strategy_info->color_mask_subject[(int)LabelModel::Blue][i].Width;
			USinfo->obstacle.Height = strategy_info->color_mask_subject[(int)LabelModel::Blue][i].Height;
			USinfo->obstacle.size = strategy_info->color_mask_subject[(int)LabelModel::Blue][i].size;
			USinfo->seeobsflag = true;
		}
		else  
		{
			USinfo->seeobsflag = false;
		}
	}
	if (USinfo->seeobsflag)
	{
		CheckAvoidObs();
	}
	else
	{
		USinfo->Initialize_Obstacle();
		USinfo->avoidobsflag = false;
	}
	SendObstacleData();
	//---------------Soccer------------------
	USinfo->Initialize_Soccer();	
	if (strategy_info->color_mask_subject_cnts[(int)LabelModel::Yellow] == 0)
	{
		USinfo->seesoccer = false;
	}
	USinfo->soccorcnt = 0;
	for (int i = 0; i < strategy_info->color_mask_subject_cnts[(int)LabelModel::Yellow]; i++)				//數某個黃模
	{
		if (strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].size > 300 && strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].Width > 15 \
			&& strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].Height > 15 && strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].size < 4500)		//黃色大小>300 && 寬度>15 && 高度>15 && 大小<4500
		{
			if ((strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].Width <= strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].Height * 1.5 \
				&& strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].Height <= strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].Width*1.5)) 		//黃色寬度<=高度*1.5 && 高度<=寬度*1.5  圓形判斷
			{
				USinfo->soccer.X = strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].X;  //x平均
				USinfo->soccer.Y = strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].Y;	 //y平均
				USinfo->soccer.XMin = strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].XMin;
				USinfo->soccer.YMin = strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].YMin;
				USinfo->soccer.XMax = strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].XMax;
				USinfo->soccer.YMax = strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].YMax;
				USinfo->soccer.Width = strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].Width;
				USinfo->soccer.Height = strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].Height;
				USinfo->soccer.size = strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].size;
				USinfo->soccorcnt = USinfo->soccorcnt+1;	//確保只抓到一顆
				USinfo->seesoccer = true;
			}
		}
		else if (strategy_info->color_mask_subject[(int)LabelModel::Yellow][i].size < USinfo->imnoisesize.soccer && USinfo->soccer.size == USinfo->imnoisesize.soccer \
			|| (USinfo->HeadHorizontal >= USinfo->upMax && USinfo->soccer.size == USinfo->imnoisesize.soccer)) 		//
		{
			USinfo->seesoccer = false;
		}
	}
	if(USinfo->soccorcnt > 2)
	{
		USinfo->soccorcnt = 0;
		USinfo->Initialize_Soccer();
		USinfo->seesoccer = false;
	}
	SendSoccerData();
}

void KidsizeImage::CheckAvoidObs()
{
    if((USinfo->obstacle.size > 9000 && USinfo->obstacle.size < 100000 && USinfo->VerticalHeadPosition < 1530 && USinfo->HorizontalHeadPosition < 2200 && USinfo->HorizontalHeadPosition > 1900) \
    	|| (USinfo->obstacle.YMax>110 && USinfo->VerticalHeadPosition <1500))
    {
		USinfo->avoidobsflag = true;
    }
}

void KidsizeImage::MoveHead(HeadMotorID ID, int Position, int Speed)
{
    ros_com->sendHeadMotor(ID,Position,Speed);
	tool->Delay(50);
    if(ID == HeadMotorID::HorizontalID)
    {
        USinfo->HorizontalHeadPosition = Position;
    }
    else
    {
        USinfo->VerticalHeadPosition = Position;
    }
}

void KidsizeImage::FindsoccerHead()
{
    switch (USinfo->HeadState)
    {
	    case etNear:       //0
	        MoveHead(HeadMotorID::VerticalID, 1880 ,300);       
	        break;
	    case etMiddle:
			MoveHead(HeadMotorID::VerticalID, 1560 ,300);
			break;
		case etClose:      //1
	        MoveHead(HeadMotorID::VerticalID, 1250 ,300);       
	        break;
    }
    switch(USinfo->HeadTurnSide)
    {
	    case TurnRight:
	        if (((USinfo->HorizontalHeadPosition - USinfo->HeadTurnSpeed) > USinfo->HorizontalMinAngle))
	        {          
	            MoveHead(HeadMotorID::HorizontalID, USinfo->HorizontalHeadPosition - USinfo->HeadTurnSpeed, 500);           //當下頭部轉40//速度300
	        }
	        else if(((USinfo->HorizontalHeadPosition - USinfo->HeadTurnSpeed) <= USinfo->HorizontalMinAngle))
	        {          
	            USinfo->HeadTurnSide = TurnLeft;
	            USinfo->HeadState = (USinfo->HeadState+1)%3;
	        }
	        break;
	    case TurnLeft:
	        if (((USinfo->HorizontalHeadPosition + USinfo->HeadTurnSpeed) < USinfo->HorizontalMaxAngle))
	        {           
	            MoveHead(HeadMotorID::HorizontalID, USinfo->HorizontalHeadPosition + USinfo->HeadTurnSpeed, 500);          
	        }
	        else if(((USinfo->HorizontalHeadPosition + USinfo->HeadTurnSpeed) >= USinfo->HorizontalMaxAngle))
	        {       
	            USinfo->HeadTurnSide = TurnRight;
	            USinfo->HeadState = (USinfo->HeadState+1)%3;
	        }
	        break;
    }
}

void KidsizeImage::TraceSoccerHead()
{
    USinfo->SoccerMoveX = ( USinfo->soccer.X - USinfo->SoccerVerticalBaseLine );//Ball.X是現在球的X位置，BallVerticalBaseLine是球需要對上的中心線
    USinfo->SoccerMoveY = ( USinfo->soccer.Y - USinfo->SoccerHortionalBaseLine );
    USinfo->ErrorHorizontalAngle = USinfo-> ImgHorizontalAngle*(double)USinfo->SoccerMoveX/(double)USinfo->RobotVisionWidth;//實際轉的角度
    USinfo->ErrorVerticalAngle = USinfo->ImgVerticalAngle*(double)USinfo->SoccerMoveY/(double)USinfo->RobotVisionHeight;//實際轉的角度
    if(USinfo->VerticalHeadPosition >= USinfo->KickSoccerHeadVertical+120)
    {
    	USinfo->VerticalRange = 60;
    	USinfo->HorizontalRange = 40;
    }
    else
    {
    	USinfo->VerticalRange = 0;
    	USinfo->HorizontalRange = 0;
    	USinfo->VerticalMinAngle = 1150;
	}
    if(abs(USinfo->soccer.X - USinfo->SoccerVerticalBaseLine) > USinfo->VerticalRange || abs(USinfo->soccer.Y - USinfo->SoccerHortionalBaseLine) > USinfo->HorizontalRange)
    {
    	if( USinfo->VerticalHeadPosition <= USinfo->VerticalMinAngle )
    	{
        	MoveHead(HeadMotorID::VerticalID, USinfo->VerticalMinAngle, 50);
    	}
    	else if ( USinfo->VerticalHeadPosition >= USinfo->VerticalMaxAngle )
    	{
        	MoveHead(HeadMotorID::VerticalID, USinfo->VerticalMaxAngle, 50);
    	}
	    if( USinfo->HorizontalHeadPosition <= USinfo->HorizontalMinAngle )
    	{
        	MoveHead(HeadMotorID::HorizontalID, USinfo->HorizontalMinAngle, 50);
    	}
    	else if ( USinfo->HorizontalHeadPosition >= USinfo-> HorizontalMaxAngle )
    	{
        	MoveHead(HeadMotorID::HorizontalID, USinfo->HorizontalMaxAngle, 50);
    	}
    	MoveHead(HeadMotorID::HorizontalID, USinfo->HorizontalHeadPosition - (USinfo->MotorMoveOneDegree * USinfo->ErrorHorizontalAngle * TraceDegreePercent), 100);
    	MoveHead(HeadMotorID::VerticalID, USinfo->VerticalHeadPosition - (USinfo->MotorMoveOneDegree * USinfo->ErrorVerticalAngle * TraceDegreePercent), 100);
    }
}

void KidsizeImage::PrintTerminal()
{
	ROS_INFO("USinfo->HeadHorizontal = %d", USinfo->HorizontalHeadPosition);
	ROS_INFO("USinfo->HeadVertical = %d", USinfo->VerticalHeadPosition);
	ROS_INFO("USinfo->seesoccer = %d", USinfo->seesoccer);
	ROS_INFO("USinfo->seeobsflag = %d", USinfo->seeobsflag);
	ROS_INFO("USinfo->soccer.size = %d", USinfo->soccer.size);
	ROS_INFO("USinfo->soccer.Width = %d", USinfo->soccer.Width);
	ROS_INFO("USinfo->soccer.Height = %d", USinfo->soccer.Height);
	ROS_INFO("soccer.Width/Height = %f", (float)USinfo->soccer.Width/USinfo->soccer.Height);
	ROS_INFO("soccer.Height/Width = %f", (float)USinfo->soccer.Height/USinfo->soccer.Width);
	ROS_INFO("USinfo->obstacle.XMin = %d", USinfo->soccer.XMin);
	ROS_INFO("USinfo->obstacle.XMax = %d", USinfo->soccer.XMax);
	ROS_INFO("USinfo->obstacle.YMin = %d", USinfo->soccer.YMin);
	ROS_INFO("USinfo->obstacle.YMax = %d", USinfo->soccer.YMax);
	ROS_INFO("USinfo->obstacle.size  = %d", USinfo->obstacle.size);
	ROS_INFO("USinfo->avoidobsflag = %d", USinfo->avoidobsflag);
	ROS_INFO("USinfo->soccorcnt = %d", USinfo->soccorcnt);
	ROS_INFO("________________________________________");
}
