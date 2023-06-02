// CtrlGroup.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_CTRL_GROUP_H
#define MOTOROS2_CTRL_GROUP_H


#define Q_SIZE 200

#if (DX100 || DX200 || FS100)
#define Q_LOCK_TIMEOUT 1000
#else
#define Q_LOCK_TIMEOUT 5000  //YRC1000 tick period is 0.2 ms
#endif

#define Q_OFFSET_IDX( a, b, c ) (((a)+(b)) >= (c) ) ? ((a)+(b)-(c)) \
                : ( (((a)+(b)) < 0 ) ? ((a)+(b)+(c)) : ((a)+(b)) )

#define MAX_JOINT_NAME_LENGTH               32
#define MAX_TF_FRAME_NAME_LENGTH            96

typedef struct
{
    UINT64 time;
    UCHAR frame;
    UCHAR user;
    UCHAR tool;
    LONG inc[MP_GRP_AXES_NUM];
} Incremental_data;

typedef struct
{
    SEM_ID q_lock;
    LONG cnt;
    LONG idx;
    Incremental_data data[Q_SIZE];
} Incremental_q;

// jointMotionData values are in radian and joint order in sequential order
typedef struct
{
    BOOL valid;                     // this point has valid data
    UINT64 time;                    // time in millisecond
    double pos[MP_GRP_AXES_NUM];    // position in radians
    double vel[MP_GRP_AXES_NUM];    // velocity in radians/s
} JointMotionData;

//---------------------------------------------------------------
// CtrlGroup:
// Structure containing all the data related to a control group
//---------------------------------------------------------------
typedef struct
{
    int groupNo;                                // sequence group number
    int numAxes;                                // number of axis in the control group
    MP_GRP_ID_TYPE groupId;                     // control group ID
    PULSE_TO_RAD pulseToRad;                    // conversion ratio between pulse and radian
    PULSE_TO_METER pulseToMeter;                // conversion ratio between pulse and meter (linear axis)
    FB_PULSE_CORRECTION_DATA correctionData;    // compensation for axes coupling
    MAX_INCREMENT_INFO maxInc;                  // maximum increment per interpolation cycle
    double maxSpeed[MP_GRP_AXES_NUM];           // maximum joint speed in radian/sec (rotational) or meter/sec (linear) (ROS joint-order)
    int tool;                                   // selected tool for the motion

    Incremental_q inc_q;                        // incremental queue
    UINT64 q_time;                              // time to which the queue has been processed

    JointMotionData* trajectoryIterator;        // joint motion command data in radian
    JointMotionData* prevTrajectoryIterator;    // joint motion command data in radian
    JointMotionData trajectoryToProcess[MAX_NUMBER_OF_POINTS_PER_TRAJECTORY];   // joint motion command data in radian to process

    BOOL hasDataToProcess;                      // indicates that there is data to process
    UINT64 timeLeftover_ms;                     // Time left over after reaching the end of a trajectory to complete the interpolation period
    long prevPulsePos[MAX_PULSE_AXES];          // The commanded pulse position that the trajectory starts at (Ros_MotionServer_StartTrajMode)
    AXIS_MOTION_TYPE axisType;                  // Indicates whether axis is rotary or linear
    char jointNames_userDefined[MP_GRP_AXES_NUM][MAX_JOINT_NAME_LENGTH]; //string name for each joint in 'moto' (non-sequential) joint order

    BOOL bIsBaxisSlave;                         // Indicates the B axis will automatically move to maintain orientation as other axes are moved

    MP_COORD robotCalibrationToBaseFrame;       // Transform from [BF] > [RF] or [BF] > [base track origin]

    MP_GRP_ID_TYPE baseTrackGroupId;            // ID for the base track associated with this robot (-1 if no base track)
    int baseTrackGroupIndex;                    // Group index for the base track associated with this robot (-1 if no base track)
    BASE_AXIS_INFO baseTrackInfo;               //

    JOINT_FEEDBACK_SPEED_ADDRESSES speedFeedbackRegisterAddress; //CIO address for the registers containing feedback speed

    int tidAddToIncQueue;

    //-------------------------------------------------------------------
    //Publishers
    //-------------------------------------------------------------------
    rcl_publisher_t publisherJointState;
    sensor_msgs__msg__JointState* msgJointState;

} CtrlGroup;


//---------------------------------
// External Functions Declaration
//---------------------------------

//Initialize specific control group. This should be called for each group connected to the robot
//controller in numerical order.
//  int groupNo: Zero based index of the group number (0-3)
//  BOOL bIsLastGrpToInit: TRUE if this is the final group that is being initialized. FALSE if you plan to call this function again.
//  float interpolPeriod: Value of the interpolation period (ms) for the robot controller.
extern CtrlGroup* Ros_CtrlGroup_Create(int groupNo, BOOL bIsLastGrpToInit, float interpolPeriod);
extern void Ros_CtrlGrp_Cleanup(CtrlGroup* ctrlGroup);

extern BOOL Ros_CtrlGroup_GetPulsePosCmd(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES]);
extern BOOL Ros_CtrlGroup_GetFBPulsePos(CtrlGroup* ctrlGroup, long pulsePos[MAX_PULSE_AXES]);
extern BOOL Ros_CtrlGroup_GetFBServoSpeed(CtrlGroup* ctrlGroup, long pulseSpeed[MAX_PULSE_AXES]);

extern BOOL Ros_CtrlGroup_GetTorque(CtrlGroup* ctrlGroup, double torqueValues[MAX_PULSE_AXES]);

extern BOOL Ros_CtrlGroup_GetEncoderTemperature(CtrlGroup const* const ctrlGroup, long encoderTemp[MAX_PULSE_AXES]);

extern void Ros_CtrlGroup_ConvertToRosPos(CtrlGroup* ctrlGroup, long const pulsePos[MAX_PULSE_AXES], double rosPos[MAX_PULSE_AXES]);
extern void Ros_CtrlGroup_ConvertToRosTorque(CtrlGroup* ctrlGroup, double const motoTorque[MAX_PULSE_AXES], double rosTorque[MAX_PULSE_AXES]);
extern void Ros_CtrlGroup_ConvertToMotoPos_FromSequentialOrdering(CtrlGroup* ctrlGroup, double const radPos[MAX_PULSE_AXES], long pulsePos[MAX_PULSE_AXES]);
extern void Ros_CtrlGroup_ConvertRosUnitsToMotoUnits(CtrlGroup* ctrlGroup, double const rosPos[MAX_PULSE_AXES], long motopulsePos[MAX_PULSE_AXES]);

extern UCHAR Ros_CtrlGroup_GetAxisConfig(CtrlGroup* ctrlGroup);

extern BOOL Ros_CtrlGroup_IsRobot(CtrlGroup* ctrlGroup);

extern void Ros_CtrlGroup_UpdateJointNamesInMotoOrder(CtrlGroup* ctrlGroup);

//String representation of MP_GRP_ID_TYPE enum. Use the MP_GRP_ID_TYPE enum as the index of this array.
extern const char* Ros_CtrlGroup_GRP_ID_String[];

#endif  // MOTOROS2_CTRL_GROUP_H
