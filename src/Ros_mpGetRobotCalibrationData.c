// Ros_mpGetRobotCalibrationData.c

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

#define FILENAME_RBCALIB_DAT    "RBCALIB.DAT"
#define PATH_TO_RBCALIB_DAT     "MPRAM1:0\\RBCALIB.DAT"

#if defined (DX200) || defined (FS100)
//The download and parsing of the DAT file is expensive. I only want to
//perform it once. So let's cache the data in an array.
//
//This is an array of POINTERS. We'll only allocate a slot if that calibration
//file is configured.
MP_RB_CALIB_DATA* Ros_CalibrationFiles[MAX_ROBOT_CALIBRATION_FILES];
#endif

void Ros_mpGetRobotCalibrationData_Initialize()
{
#if defined (DX200) || defined (FS100)
    int ret;
    int fd;
    BOOL bRet;

    const int SIZEOFBUFFER = 256;
    char buffer[SIZEOFBUFFER];

    rcutils_string_array_t splitSpace = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t splitComma = rcutils_get_zero_initialized_string_array();

    bzero(Ros_CalibrationFiles, sizeof(Ros_CalibrationFiles));

    //make sure to remove any old copy that may be out there
    mpRemove(PATH_TO_RBCALIB_DAT); //dont care if this fails

    //save from CMOS to DRAM
    ret = mpSaveFile(MP_DRV_ID_DRAM, "", FILENAME_RBCALIB_DAT);
    if (ret != OK)
    {
        Ros_Debug_BroadcastMsg("Couldn't save " FILENAME_RBCALIB_DAT " to DRAM drive. Error (%d)", ret);
        Ros_Debug_BroadcastMsg("/tf will be inaccurate for multi-group systems");
        mpSetAlarm(ALARM_DAT_FILE_PARSE_FAIL, "Failed to parse " FILENAME_RBCALIB_DAT, SUBCODE_DAT_FAIL_CMOS_TO_DRAM);
        return;
    }

    fd = mpOpen(PATH_TO_RBCALIB_DAT, O_RDONLY, 0);
    if (fd < 0)
    {
        Ros_Debug_BroadcastMsg("Couldn't open " FILENAME_RBCALIB_DAT " from DRAM drive. fd (%d)", fd);
        Ros_Debug_BroadcastMsg("/tf will be inaccurate for multi-group systems");
        mpSetAlarm(ALARM_DAT_FILE_PARSE_FAIL, "Failed to parse " FILENAME_RBCALIB_DAT, SUBCODE_DAT_FAIL_OPEN);
        return;
    }

    while (TRUE) //unknown file size. this will `break` when done with file
    {
        //---------------------------------------------------------------------
        bRet = FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //RBCALIB x

        if (bRet && strstr(buffer, "//RBCALIB"))
        {
            rcutils_split(buffer, ' ', g_motoros2_Allocator, &splitSpace);

            int fileNo = atoi(splitSpace.data[1]) - 1; //fileNo is zero-based, RBCALIB is one-based
            Ros_CalibrationFiles[fileNo] = (MP_RB_CALIB_DATA*)mpMalloc(sizeof(MP_RB_CALIB_DATA));

            ret = rcutils_string_array_fini(&splitSpace); RCUTILS_UNUSED(ret);

            //---------------------------------------------------------------------
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //MTOOL x

            //---------------------------------------------------------------------
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //MGROUP x,x,x,....
            rcutils_split(buffer, ' ', g_motoros2_Allocator, &splitSpace);
            rcutils_split(splitSpace.data[1], ',', g_motoros2_Allocator, &splitComma);

            for (int groupIndex = 0; groupIndex < 32; groupIndex += 1)
            {
                if (atoi(splitComma.data[groupIndex]) == 1)
                {
                    Ros_CalibrationFiles[fileNo]->m_rb.grp_no = groupIndex;
                    break;
                }
            }
            ret = rcutils_string_array_fini(&splitSpace); RCUTILS_UNUSED(ret);
            ret = rcutils_string_array_fini(&splitComma); RCUTILS_UNUSED(ret);

            //---------------------------------------------------------------------
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //MPULSE
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //MRBC1
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //MRBC2
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //MRBC3
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //STOOL x

            //---------------------------------------------------------------------
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //SGROUP x,x,x,....
            rcutils_split(buffer, ' ', g_motoros2_Allocator, &splitSpace);
            rcutils_split(splitSpace.data[1], ',', g_motoros2_Allocator, &splitComma);

            for (int groupIndex = 0; groupIndex < 32; groupIndex += 1)
            {
                if (atoi(splitComma.data[groupIndex]) == 1)
                {
                    Ros_CalibrationFiles[fileNo]->s_rb.grp_no = groupIndex;
                    break;
                }
            }
            ret = rcutils_string_array_fini(&splitSpace); RCUTILS_UNUSED(ret);
            ret = rcutils_string_array_fini(&splitComma); RCUTILS_UNUSED(ret);

            //---------------------------------------------------------------------
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //SPULSE
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //SSTC1
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //SSTC2
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //SSTC3

            //---------------------------------------------------------------------
            FileUtilityFunctions_ReadLine(fd, buffer, SIZEOFBUFFER); //  //SRANG x,y,z,rx,ry,rz
            rcutils_split(buffer, ' ', g_motoros2_Allocator, &splitSpace);
            rcutils_split(splitSpace.data[1], ',', g_motoros2_Allocator, &splitComma);

            Ros_CalibrationFiles[fileNo]->pos_uow[0] = (int)(atof(splitComma.data[0]) * 1000);
            Ros_CalibrationFiles[fileNo]->pos_uow[1] = (int)(atof(splitComma.data[1]) * 1000);
            Ros_CalibrationFiles[fileNo]->pos_uow[2] = (int)(atof(splitComma.data[2]) * 1000);

            Ros_CalibrationFiles[fileNo]->ang_uow[0] = (int)(atof(splitComma.data[3]) * 10000);
            Ros_CalibrationFiles[fileNo]->ang_uow[1] = (int)(atof(splitComma.data[4]) * 10000);
            Ros_CalibrationFiles[fileNo]->ang_uow[2] = (int)(atof(splitComma.data[5]) * 10000);

            ret = rcutils_string_array_fini(&splitSpace); RCUTILS_UNUSED(ret);
            ret = rcutils_string_array_fini(&splitComma); RCUTILS_UNUSED(ret);
        }
        else
            break;
    }

    mpClose(fd);
    mpRemove(PATH_TO_RBCALIB_DAT); //dont care if this fails
#endif
}

void Ros_mpGetRobotCalibrationData_Cleanup()
{
#if defined (DX200) || defined (FS100)
    for (int i = 0; i < MAX_ROBOT_CALIBRATION_FILES; i += 1)
    {
        if (Ros_CalibrationFiles[i])
            mpFree(Ros_CalibrationFiles[i]);
    }
#endif
}

LONG Ros_mpGetRobotCalibrationData(ULONG file_no, MP_RB_CALIB_DATA *rData)
{
#if defined (YRC1000) || defined (YRC1000u)
    // on these controllers we can forward to mpGetRobotCalibrationData(..)
    return mpGetRobotCalibrationData(file_no, rData);

#elif defined (DX200) || defined (FS100)

    if (Ros_CalibrationFiles[file_no] != NULL) //if this calibration file exists
    {
        memcpy(rData, Ros_CalibrationFiles[file_no], sizeof(MP_RB_CALIB_DATA));
        return OK;
    }
    else
        return NG;

#else
    #error Ros_mpGetRobotCalibrationData: unsupported platform

#endif
}
