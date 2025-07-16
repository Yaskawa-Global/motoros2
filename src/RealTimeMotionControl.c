
#include "MotoROS.h"

void MotionControl_RtIncMoveLoopStart()
{
    MP_EXPOS_DATA moveData;
    int i;

    bzero(&moveData, sizeof(moveData));

    for (i = 0; i < g_Ros_Controller.numGroup; i++)
    {
        moveData.ctrl_grp |= (0x01 << i);
        moveData.grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(g_Ros_Controller.ctrlGroups[i]);

        ctrlGrpData.sCtrlGrp = g_Ros_Controller.ctrlGroups[i]->groupId;
        mpGetPulsePos(&ctrlGrpData, &prevPulsePosData[i]);
    }
}
