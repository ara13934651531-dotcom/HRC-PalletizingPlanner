
#pragma once

typedef int                                RTS_BOOL;
typedef signed char             RTS_IEC_BOOL;
typedef signed short int     RTS_IEC_INT;
typedef double                       RTS_IEC_LREAL;
typedef signed long int       RTS_IEC_LINT;
typedef signed int                 RTS_IEC_DINT;

typedef struct tagMC_COORD_REF
{
	RTS_IEC_LREAL X;
	RTS_IEC_LREAL Y;
	RTS_IEC_LREAL Z;
	RTS_IEC_LREAL A;
	RTS_IEC_LREAL B;
	RTS_IEC_LREAL C;

}MC_COORD_REF;

typedef enum {
    Elfin = 0,                      // 选择 Elfin 构型的运动学模型
    UR = 1,                         // 选择 UR 构型的运动学模型
}robotType;

typedef enum {
    Ball = 1,                        // 选择球碰撞模型
    Capsule = 2,                     // 选择胶囊碰撞模型
    Lozenge = 3,                     // 选择棱体碰撞模型
    Plane = 4                        // 选择平面碰撞模型
}collisionModelType;

typedef enum {        
    WorkBench2 = -20,          // 工作台2 (相对Frame 0定义)
    WorkBench1 = -21,          // 工作台1 (相对Frame 0定义)
    WorkBench0 = -19,          // 基座安装台 (相对Frame 0定义)
    NO_COLLISION = 0,
    Base = 1,                // 基座 (相对Frame 1定义)
    LowerArm = 2,            // 下臂 (相对Frame 2定义)  
    Elbow = 3,               // 肘部 (相对Frame 3定义)  
    UpperArm = 4,            // 上臂 (相对Frame 4定义) 
    Wrist = 5,               // 腕部 (相对Frame 5定义)  
    Tool1 = 6,               // 末端工具1 (相对Tool系定义)
    Tool2 = 7,               // 末端工具2 (相对Tool系定义)

    Plane1 = 10,
    Plane2 = 11,
    Plane3 = 12,
    Plane4 = 13,
    Plane5 = 14,
    Plane6 = 15,
    Plane7 = 16,
    Plane8 = 17,

    Obb1 = 20,
    Obb2 = 21,
    Obb3 = 22,
    Obb4 = 23,
    Obb5 = 24,
}collisionModelIndex;