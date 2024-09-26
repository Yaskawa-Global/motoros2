//ServiceJobList.c

// SPDX-FileCopyrightText: 2024, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2024, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceListInformJobs;

ServiceListInformJobs_Messages g_messages_ListInformJobs;

typedef motoros2_interfaces__srv__ListInformJobs_Response ListInformJobsResponse;


static const size_t RESP_NAMES_LIST_INITIAL_SIZE = 1;
static const size_t MAX_NUMBER_OF_JOBS_TO_REPORT = 1024;

#define MSG_RC(x) (motoros2_interfaces__msg__InformJobCrudResultCodes__RC_ ## x)
#define MSG_RC_STR(x) (motoros2_interfaces__msg__InformJobCrudResultCodes__STR_ ## x)


void Ros_ServiceListInformJobs_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_list_jobs_init);

    Ros_Debug_BroadcastMsg("%s: maximum nr of jobs supported: %d",
        __func__, MAX_NUMBER_OF_JOBS_TO_REPORT);

    const rosidl_service_type_support_t* type_support =
        ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, ListInformJobs);

    rcl_ret_t ret = rclc_service_init_default(
        &g_serviceListInformJobs, &g_microRosNodeInfo.node, type_support, SERVICE_NAME_LIST_INFORM_JOBS);
    motoRosAssert_withMsg(ret == RCL_RET_OK,
        SUBCODE_FAIL_INIT_SERVICE_LIST_INFORM_JOBS, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_ListInformJobs.response.message);
    //NOTE: if/when needed, this will be resized
    rosidl_runtime_c__String__Sequence__init(
        &g_messages_ListInformJobs.response.names, RESP_NAMES_LIST_INITIAL_SIZE);
    g_messages_ListInformJobs.response.names.size = 0;

    MOTOROS2_MEM_TRACE_REPORT(svc_list_jobs_init);
}

void Ros_ServiceListInformJobs_Cleanup()
{
    rcl_ret_t ret;
    MOTOROS2_MEM_TRACE_START(svc_list_jobs_fini);

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_LIST_INFORM_JOBS);
    ret = rcl_service_fini(&g_serviceListInformJobs, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_LIST_INFORM_JOBS " service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_ListInformJobs.response.message);
    rosidl_runtime_c__String__Sequence__fini(&g_messages_ListInformJobs.response.names);

    MOTOROS2_MEM_TRACE_REPORT(svc_list_jobs_fini);
}

void Ros_ServiceListInformJobs_Trigger(const void* request_msg, void* response_msg)
{
    RCL_UNUSED(request_msg);
    ListInformJobsResponse* response = (ListInformJobsResponse*)response_msg;

    Ros_Debug_BroadcastMsg("%s: enter", __func__);

    //make sure we don't accidentally return old results (in case of errors fi)
    rosidl_runtime_c__String__assign(&response->message, "");
    response->result_code = 0;
    response->names.size = 0;

    //retrieve list of jobs (".JBI")
    Ros_Debug_BroadcastMsg("%s: updating job file list", __func__);
    LONG status = mpRefreshFileList(MP_EXT_ID_JBI);
    if (status != OK)
    {
        Ros_Debug_BroadcastMsg("%s: error refreshing job list", __func__);
        rosidl_runtime_c__String__assign(&response->message, MSG_RC_STR(ERR_REFRESHING_JOB_LIST));
        response->result_code = MSG_RC(ERR_REFRESHING_JOB_LIST);
        goto DONE;
    }

    //see how many jobs there are, and if necessary, resize list of names in service response
    size_t job_count = mpGetFileCount();
    if (job_count < 0)
    {
        Ros_Debug_BroadcastMsg("%s: error retrieving file count: %d", __func__, job_count);
        rosidl_runtime_c__String__assign(&response->message, MSG_RC_STR(ERR_RETRIEVING_FILE_COUNT));
        response->result_code = MSG_RC(ERR_RETRIEVING_FILE_COUNT);
        goto DONE;
    }
    Ros_Debug_BroadcastMsg("%s: found %d jobs", __func__, job_count);

    //there is a limit to the number of jobs we support
    if (job_count > MAX_NUMBER_OF_JOBS_TO_REPORT)
    {
        Ros_Debug_BroadcastMsg("%s: too many jobs, aborting", __func__);
        rosidl_runtime_c__String__assign(&response->message, MSG_RC_STR(ERR_TOO_MANY_JOBS));
        response->result_code = MSG_RC(ERR_TOO_MANY_JOBS);
        goto DONE;
    }

    //resize
    if (job_count > response->names.capacity)
    {
        Ros_Debug_BroadcastMsg("%s: resize needed: current: %d; need: %d",
            __func__, response->names.capacity, job_count);
        rosidl_runtime_c__String__Sequence__fini(&response->names);
        //TODO(gavanderhoorn): handle errors
        rosidl_runtime_c__String__Sequence__init(&response->names, job_count);
    }

    //copy all jobs on the list to the response
    //name, plus extension, plus terminating null
#define JOB_EXT_LEN (4)
#define MAX_JOB_NAME_LEN_WITH_EXT (MAX_JOB_NAME_LEN + JOB_EXT_LEN + 1)
    char name_buf[MAX_JOB_NAME_LEN_WITH_EXT] = { 0 };
    for (size_t i = 0; i < job_count; ++i)
    {
        //retrieve job name from internal list
        bzero(name_buf, MAX_JOB_NAME_LEN_WITH_EXT);
        status = mpGetFileName(i, name_buf);
        if (status != OK)
        {
            Ros_Debug_BroadcastMsg("%s: failed retrieving job %d from list", __func__, i);
            rosidl_runtime_c__String__assign(&response->message, MSG_RC_STR(ERR_RETRIEVING_JOB_NAME_FROM_LIST));
            response->result_code = MSG_RC(ERR_RETRIEVING_JOB_NAME_FROM_LIST);
            goto DONE;
        }

        //determine length without extension (which is 4 less than the full name)
        size_t name_len = Ros_strnlen(name_buf, MAX_JOB_NAME_LEN_WITH_EXT);
        if (name_len <= JOB_EXT_LEN)
        {
            Ros_Debug_BroadcastMsg("%s: job name '%s' too short (%d chars), can't remove extension",
                __func__, name_buf, name_len);
        }
        else
        {
            name_len -= JOB_EXT_LEN;
        }

        //add it to the response
        if (!rosidl_runtime_c__String__assignn(&response->names.data[i], name_buf, name_len))
        {
            Ros_Debug_BroadcastMsg("%s: failed adding job name %d to response, aborting", __func__, i);
            rosidl_runtime_c__String__assign(&response->message, MSG_RC_STR(ERR_APPENDING_JOB_NAME_TO_SVC_RESPONSE));
            response->result_code = MSG_RC(ERR_APPENDING_JOB_NAME_TO_SVC_RESPONSE);
            goto DONE;
        }
    }
#undef JOB_EXT_LEN
#undef MAX_JOB_NAME_LEN_WITH_EXT

    //finally update nr of items in list
    response->names.size = job_count;

    //done, report to client
    Ros_Debug_BroadcastMsg("%s: returning %d job names", __func__, response->names.size);
    rosidl_runtime_c__String__assign(&response->message, MSG_RC_STR(OK));
    response->result_code = MSG_RC(OK);

DONE:
    Ros_Debug_BroadcastMsg("%s: exit", __func__);
}
