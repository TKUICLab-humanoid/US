#define TurnLeft 1
#define TurnRight 0
#define etNear 0
#define etMiddle 1
#define etClose 2
#define etFar 0
#define TraceDegreePercent 0.2

enum StrategymainState { Initial = 0, Find_Soccer, Trace_Soccer};
enum BodyActionState { SmallMove = 0, Move, BigMove, SmallLMove, LMove, SmallRMove, RMove, SmallLRotation, LRotation, BigLRotation, SmallRRotation, RRotation, BigRRotation, OneStep, Back,
  Continuous, ContinuousStart, ContinuousStop, ContinuousShift, ContinuousMaxValue, ContinuousMinValue, ContinuousLMove, ContinuousRMove, ContinuousSmallLRotation, ContinuousBigLRotation, \
  ContinuousSmallRRotation, ContinuousBigRRotation, ContinuousKickBallLRotation, ContinuousKickBallRRotation, ContinuousLShiftRotation, ContinuousRShiftRotation,ContinuousLRotation,\
  ContinuousRRotation, ContinuousOneStep,ContinuousRStart,ContinuousLStart, ContinuousSmallRMove,ContinuousSmallLMove,ContinuousBack };
enum ContinuousValseStates {ShiftX, ShiftY, ShiftTheta};

struct Datainfo
{
	int X, Y;
	int XMin, XMax;
	int YMin, YMax;
	int Width, Height;
	int size;
};

struct imnoiseSize
{
	int soccer;
	int goal;
	int obstacle;
};

struct BodyActionValue
{
	int X, Y, Z, theta, Mode, IMU;
};

class USInfo
{
public:
	USInfo();
	~USInfo();
public:
	
	void Initialize_tracksoccer();
	void Initialize_Obstacle();
	void Initialize_Soccer();
	
public:

	//------------IMU-------------
	int IMU_Yaw_initial;
	int IMU_Pitch_initial ;
	//----------------tracksoccer---------------------------
	bool avoidobsflag;
	int HeadState;
    int HeadTurnSide;
    int HeadTurnSpeed;
    int HeadHorizontalIni;
    int HeadVerticalIni;
    int HorizontalHeadPosition;
    int HorizontalMaxAngle;
    int HorizontalMinAngle;
    int HorizontalRange;
    int KickSoccerHeadVertical;
    int VerticalHeadPosition;
    int VerticalMaxAngle;
    int VerticalMinAngle;
    int VerticalRange;
    int RobotVisionHeight;
    int RobotVisionWidth;
    int soccorcnt;
    int SoccerHortionalBaseLine;
    int SoccerVerticalBaseLine;
	int cannotsee;
    double ErrorHorizontalAngle;
    double ErrorVerticalAngle;
    double ImgHorizontalAngle;
    double ImgVerticalAngle;
	double MotorMoveOneDegree;
	double SoccerMoveX;
	double SoccerMoveY;

	//------.ini------
	bool seesoccer;
	bool seeobsflag;
	int HeadHorizontal;
	int HeadVertical;
	int upMax;

	Datainfo soccer;
	Datainfo obstacle;
	imnoiseSize imnoisesize;
	//------------------------strategymain-------------------------------
public:
	ContinuousValseStates ContinuousValseState;
	StrategymainState BodyStrategySelect;

	bool AimGoalFlag;
	bool ContinuousFlag;
	bool ContinuousLRotationFlag;
	bool ContinuousRRotationFlag;
	int FixHead_LB_Limit;
	int FixHead_RB_Limit;
	int MiddleDecade;
    int NearDecade;
    double ContinuousTimeError;
	double ContinuousTimeRatio;
    double OneSteptoContinuousRatio;
    double StartPositionError;
	//------.ini------
	BodyActionValue MoveValue[ContinuousBack];
};

extern USInfo* USinfo;
