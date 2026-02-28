
#pragma once

#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

typedef int8_t			RTS_IEC_BOOL;
typedef int			RTS_IEC_INT;
typedef int64_t			RTS_IEC_LINT;
typedef double		RTS_IEC_LREAL;
typedef uint16_t		RTS_IEC_UINT;
typedef int32_t			RTS_IEC_DINT;


typedef struct tagMC_COORD_REF
{
	RTS_IEC_LREAL X;
	RTS_IEC_LREAL Y;
	RTS_IEC_LREAL Z;
	RTS_IEC_LREAL A;
	RTS_IEC_LREAL B;
	RTS_IEC_LREAL C;

}MC_COORD_REF;

typedef struct tagweaveParams
{
	RTS_IEC_LREAL frequency;		/* 频率 */
	RTS_IEC_LREAL amplitude;		/* 振幅 */
	RTS_IEC_INT weaveFrameType;		/* 摆焊平面选择，0:由工具坐标系z方向和焊道方向构成坐标系，1:由工具坐标系z方向和y方向构成坐标系 */
	RTS_IEC_LREAL rightDwell;		/* 右停留时间 */
	RTS_IEC_LREAL leftDwell;		/* 左停留时间 */
	RTS_IEC_LREAL LPatternAngle;		/* L形角度； */
	RTS_IEC_LREAL elevation;		/* 仰角 */
	RTS_IEC_LREAL azimuth;		/* 方向角 */
	RTS_IEC_LREAL centreRise;		/* 中心隆起 */
	RTS_IEC_LREAL radius;		/* 半径 */
} weaveParams;


typedef struct tagmotionStateST
{
	RTS_IEC_LREAL jointPosition[10];
	RTS_IEC_LREAL jointVelocity[10];
	RTS_IEC_LREAL jointAcceleration[10];
	RTS_IEC_LREAL cartPose[6];
	RTS_IEC_LREAL cartFusePose[6];
	RTS_IEC_LREAL cartVelocity[6];
	RTS_IEC_LREAL pathVelocity[2];
} motionStateST;

typedef struct tagmotionSettingParamsST
{
	RTS_IEC_LREAL timeCycle;
	RTS_IEC_LREAL override;
	RTS_IEC_LREAL kinParams[10];		/* kinrmatics parameters. */
	RTS_IEC_LREAL toolOffset[6];
	RTS_IEC_LREAL upperJointLimits[10];
	RTS_IEC_LREAL lowerJointLimits[10];
	RTS_IEC_LREAL jointMaxVelocity[10];
	RTS_IEC_LREAL jointMaxAcceleration[10];
	RTS_IEC_LREAL jointMaxJerk[10];
	RTS_IEC_LREAL jointStoppingAcceleration[10];
	RTS_IEC_LREAL jointStoppingJerk[10];
	RTS_IEC_LREAL linearStoppingAcceleration[2];
	RTS_IEC_LREAL linearStoppingJerk[2];
} motionSettingParamsST;


typedef struct tagcobotParams
{
	RTS_IEC_LREAL kinParams[10];		/* kinrmatics parameters. */
	RTS_IEC_LREAL actuatorDamp[6];
	RTS_IEC_LREAL torqueConstant[6];
	RTS_IEC_LREAL gearRatio[6];
	RTS_IEC_LREAL maxEfficiency[6];
	RTS_IEC_LREAL maxActuatorCurrents[6];
	RTS_IEC_LREAL rotationofMounting;		/* rotate robot base mounting angles(deg) */
	RTS_IEC_LREAL tiltofMounting;		/* tilt robot base mounting angles(deg) */
	RTS_IEC_LREAL upperJointLimits[6];
	RTS_IEC_LREAL lowerJointLimits[6];
	RTS_IEC_LREAL dynamicsParams[78];
	RTS_IEC_LREAL collisionStopThresholds[6];		/* joint collisiton threshold; */
	RTS_IEC_LREAL collisionStopInMomentumThresholds[6];		/* joint collisiton threshold; */
	RTS_IEC_LREAL assistiveModeCollisionStopThresholds[6];
	RTS_IEC_LREAL frictionCompensatoryFactor[6];		/* default setting: J1="0.5" J2="0.5" J3="0.4" J4="0.6" J5="0.7" J6="0.7" */
	RTS_IEC_LREAL frictionCompensatoryFactorII[6];		/* default seting [1.0]; */
	RTS_IEC_LREAL lowVelocityThreshold[6];		/* decide whether the joints are in low velocity status; */
	RTS_IEC_LREAL maxJointVelocity[6];		/*  */
	RTS_IEC_LREAL maxPowerConstraint;		/* power */
	RTS_IEC_LREAL maxMomentumConstraint;		/* momentum */
	RTS_IEC_LREAL jointPowerConstraint[6];		/* power of joints; */
	RTS_IEC_LREAL distanceLimitInDriveBack;		/* allowing joint motion(degree,  °) in drive back mode; */
	RTS_IEC_LREAL timeDurationInDriveBack;		/* allowing time duration(second, s) in drive back mode; */
	RTS_IEC_LREAL mass;
	RTS_IEC_LREAL centerofMass[3];		/* (millimeter) */
	RTS_IEC_BOOL isAssistiveMode;
	RTS_IEC_INT drivebackMode;
	RTS_IEC_LREAL assistiveCheckTime;		/* the start time for check robot status in assistive mode, second ,s; */
} cobotParams;