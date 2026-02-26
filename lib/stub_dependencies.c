/**
 * @file stub_dependencies.c
 * @brief 缺失依赖库的 stub 实现
 * 
 * libCmpRML.so 依赖 libCmpHansFreeDriveMotion.so, libCmpHansAlgorithmLib.so, 
 * libwl.so 用于力控拖拽/焊接/位姿跟踪等高级功能。
 * 码垛仿真只使用基础运动功能, 这些高级功能以 stub 方式提供空实现。
 */

// ============================================================================
// libCmpHansFreeDriveMotion.so stubs (力控拖拽功能)
// ============================================================================

void resetFreeDriveMotion(void) {}
void setFreeDriveDHParameters(void) {}
void setFreeDriveTimeParams(void) {}
void setFreeDriveCompensatingForce(void) {}
void setFreeDriveFilterSwitch(void) {}
void setFreeDriveWrenchThreshold(void) {}
void setFreeDriveForceControlFrame(void) {}
int  getFreeDriveSpeedMode(void) { return 0; }
void setFreeDriveStartPosition(void) {}
void setFreeDriveFTSensorPosition(void) {}
void setFreeDriveVariableAdmitMassAndDamp(void) {}
void setFreeDriveVariableAdmitDampLimit(void) {}
void setFreeDrivePhysicParams(void) {}
void setFreeDriveToolFrameXYZ(void) {}
int  getFreeDriveComplianceFactor(void) { return 0; }
void setFreeDriveComplianceFactor(void) {}
void setFreeDriveToolFrameRPY(void) {}
void setFreeDriveJointBounds(void) {}
void setFreeDriveRobotType(void) {}
void setFreeDriveVirtualWallThicknessFactor(void) {}
void setFreeDriveCartSpeedLimit(void) {}
void setFreeDriveWrenchThreshold4StartupSafety(void) {}
void setFreeDriveHandPointPosition(void) {}
void setFreeDriveMotionMode(void) {}
void setFreeDriveLoadMass(void) {}
void setFreeDriveSpeedMode(void) {}
void updateFreeDriveRT(void) {}
void updateFreeDriveWrenchValue(void) {}

// ============================================================================
// libCmpHansAlgorithmLib.so stubs (焊接/跟踪算法)
// ============================================================================

void calculateWeaveOffsetPoseAPI(void) {}
void initServoJManager(void) {}
void pushServoJTargets(void) {}
void updateServoJManagerState(void) {}
void setServoJManagerLimits(void) {}
void initPoseTracking(void) {}
void updatePoseTrackingSensor(void) {}
int  getPoseTrackingStatus(void) { return 0; }
void setPoseTrackingTarget(void) {}
void setPoseTrackingPosRotPIParams(void) {}
void setPoseTrackingStopDuration(void) {}
void startPoseTrackingAPI(void) {}
void setPoseTrackingMotionLimits(void) {}
int  getPoseTrackingCommandVel(void) { return 0; }
void setWeaveArcPathParamsAPI(void) {}
void setWeavePathSpeedAPI(void) {}
void startWeaveWeldWeldAPI(void) {}
void setArcPathBlendingPointParamsAPI(void) {}

// ============================================================================
// libwl.so stubs (CoDeSys/PLC runtime)
// ============================================================================

// 无需额外stub, 上述已覆盖
