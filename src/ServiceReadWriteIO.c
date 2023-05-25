//ServiceReadWriteIO.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

typedef enum
{
    // TODO(gavanderhoorn): not too nice, but at least we shorten the names a little
    // TODO(gavanderhoorn): generalise the result codes more, so they are not Yaskawa-specific any more
    IO_RESULT_OK = motoros2_interfaces__msg__IoResultCodes__OK,
    IO_RESULT_READ_ADDRESS_INVALID = motoros2_interfaces__msg__IoResultCodes__READ_ADDRESS_INVALID,
    IO_RESULT_WRITE_ADDRESS_INVALID = motoros2_interfaces__msg__IoResultCodes__WRITE_ADDRESS_INVALID,
    IO_RESULT_WRITE_VALUE_INVALID = motoros2_interfaces__msg__IoResultCodes__WRITE_VALUE_INVALID,
    IO_RESULT_READ_API_ERROR = motoros2_interfaces__msg__IoResultCodes__READ_API_ERROR,
    IO_RESULT_WRITE_API_ERROR = motoros2_interfaces__msg__IoResultCodes__WRITE_API_ERROR
} IoResultCodes;

typedef enum
{
    IO_ACCESS_BIT,
    IO_ACCESS_GROUP,
    IO_ACCESS_REGISTER
} IoAccessSize;

static BOOL Ros_IoServer_IsValidReadAddress(UINT32 address, IoAccessSize size);
static BOOL Ros_IoServer_IsValidWriteAddress(UINT32 address, IoAccessSize size);
static BOOL Ros_IoServer_IsValidWriteValue(UINT32 value, IoAccessSize size);
static const char* const Ros_IoServer_ResultCodeToStr(UINT32 resultCode);

//**********************************************************
#if DX100
#define GENERALINMIN (10)
#define GENERALINMAX (2567)

#define GENERALOUTMIN (10010)
#define GENERALOUTMAX (12567)

#define EXTERNALINMIN (20010)
#define EXTERNALINMAX (22567)

#define NETWORKINMIN (25010)
#define NETWORKINMAX (27567)

#define NETWORKOUTMIN (35010)
#define NETWORKOUTMAX (37567)

#define EXTERNALOUTMIN (30010)
#define EXTERNALOUTMAX (32567)

#define SPECIFICINMIN (40010)
#define SPECIFICINMAX (41607)

#define SPECIFICOUTMIN (50010)
#define SPECIFICOUTMAX (52007)

#define IFPANELMIN (60010)
#define IFPANELMAX (60647)

#define AUXRELAYMIN (70010)
#define AUXRELAYMAX (79997)

#define CONTROLSTATUSMIN (80010)
#define CONTROLSTATUSMAX (80647)

#define PSEUDOINPUTMIN (82010)
#define PSEUDOINPUTMAX (82207)

#define REGISTERMIN (1000000)
#define REGISTERMAX_READ (1000999)
#define REGISTERMAX_WRITE (1000559)

//**********************************************************
#elif FS100
#define GENERALINMIN (10)
#define GENERALINMAX (1287)

#define GENERALOUTMIN (10010)
#define GENERALOUTMAX (11287)

#define EXTERNALINMIN (20010)
#define EXTERNALINMAX (21287)

#define NETWORKINMIN (25010)
#define NETWORKINMAX (26287)

#define NETWORKOUTMIN (35010)
#define NETWORKOUTMAX (36287)

#define EXTERNALOUTMIN (30010)
#define EXTERNALOUTMAX (31287)

#define SPECIFICINMIN (40010)
#define SPECIFICINMAX (41607)

#define SPECIFICOUTMIN (50010)
#define SPECIFICOUTMAX (52007)

#define IFPANELMIN (60010)
#define IFPANELMAX (60647)

#define AUXRELAYMIN (70010)
#define AUXRELAYMAX (79997)

#define CONTROLSTATUSMIN (80010)
#define CONTROLSTATUSMAX (80647)

#define PSEUDOINPUTMIN (82010)
#define PSEUDOINPUTMAX (82207)

#define REGISTERMIN (1000000)
#define REGISTERMAX_READ (1000999)
#define REGISTERMAX_WRITE (1000559)

//**********************************************************
#elif DX200
#define GENERALINMIN (10)
#define GENERALINMAX (5127)

#define GENERALOUTMIN (10010)
#define GENERALOUTMAX (15127)

#define EXTERNALINMIN (20010)
#define EXTERNALINMAX (25127)

#define NETWORKINMIN (27010)
#define NETWORKINMAX (29567)

#define NETWORKOUTMIN (37010)
#define NETWORKOUTMAX (39567)

#define EXTERNALOUTMIN (30010)
#define EXTERNALOUTMAX (35127)

#define SPECIFICINMIN (40010)
#define SPECIFICINMAX (41607)

#define SPECIFICOUTMIN (50010)
#define SPECIFICOUTMAX (53007)

#define IFPANELMIN (60010)
#define IFPANELMAX (60647)

#define AUXRELAYMIN (70010)
#define AUXRELAYMAX (79997)

#define CONTROLSTATUSMIN (80010)
#define CONTROLSTATUSMAX (82007)

#define PSEUDOINPUTMIN (82010)
#define PSEUDOINPUTMAX (82207)

#define REGISTERMIN (1000000)
#define REGISTERMAX_READ (1000999)
#define REGISTERMAX_WRITE (1000559)

//**********************************************************
#elif YRC1000
#define GENERALINMIN (10)
#define GENERALINMAX (5127)

#define GENERALOUTMIN (10010)
#define GENERALOUTMAX (15127)

#define EXTERNALINMIN (20010)
#define EXTERNALINMAX (25127)

#define NETWORKINMIN (27010)
#define NETWORKINMAX (29567)

#define NETWORKOUTMIN (37010)
#define NETWORKOUTMAX (39567)

#define EXTERNALOUTMIN (30010)
#define EXTERNALOUTMAX (35127)

#define SPECIFICINMIN (40010)
#define SPECIFICINMAX (42567)

#define SPECIFICOUTMIN (50010)
#define SPECIFICOUTMAX (55127)

#define IFPANELMIN (60010)
#define IFPANELMAX (60647)

#define AUXRELAYMIN (70010)
#define AUXRELAYMAX (79997)

#define CONTROLSTATUSMIN (80010)
#define CONTROLSTATUSMAX (85127)

#define PSEUDOINPUTMIN (87010)
#define PSEUDOINPUTMAX (87207)

#define REGISTERMIN (1000000)
#define REGISTERMAX_READ (1000999)
#define REGISTERMAX_WRITE (1000559)

//**********************************************************
#elif YRC1000u
#define GENERALINMIN (10)
#define GENERALINMAX (5127)

#define GENERALOUTMIN (10010)
#define GENERALOUTMAX (15127)

#define EXTERNALINMIN (20010)
#define EXTERNALINMAX (21287)

#define NETWORKINMIN (27010)
#define NETWORKINMAX (29567)

#define NETWORKOUTMIN (37010)
#define NETWORKOUTMAX (39567)

#define EXTERNALOUTMIN (30010)
#define EXTERNALOUTMAX (31287)

#define SPECIFICINMIN (40010)
#define SPECIFICINMAX (42567)

#define SPECIFICOUTMIN (50010)
#define SPECIFICOUTMAX (55127)

#define IFPANELMIN (60010)
#define IFPANELMAX (60647)

#define AUXRELAYMIN (70010)
#define AUXRELAYMAX (79997)

#define CONTROLSTATUSMIN (80010)
#define CONTROLSTATUSMAX (85127)

#define PSEUDOINPUTMIN (87010)
#define PSEUDOINPUTMAX (87207)

#define REGISTERMIN (1000000)
#define REGISTERMAX_READ (1000999)
#define REGISTERMAX_WRITE (1000559)

//* end of controller IO range defs ************************
#endif

#define QUANTITY_BIT	(1)
#define QUANTITY_BYTE	(8)

rcl_service_t g_serviceReadSingleIO;
rcl_service_t g_serviceReadGroupIO;
rcl_service_t g_serviceWriteSingleIO;
rcl_service_t g_serviceWriteGroupIO;
rcl_service_t g_serviceReadMRegister;
rcl_service_t g_serviceWriteMRegister;

ServiceReadWriteIO_Messages g_messages_ReadWriteIO;

void Ros_ServiceReadWriteIO_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_rw_io_init);

    rcl_ret_t ret;

    const rosidl_service_type_support_t* t_supp_read_single_io  = ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, ReadSingleIO);
    const rosidl_service_type_support_t* t_supp_read_group_io   = ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, ReadGroupIO);
    const rosidl_service_type_support_t* t_supp_write_single_io = ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, WriteSingleIO);
    const rosidl_service_type_support_t* t_supp_write_group_io  = ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, WriteGroupIO);
    const rosidl_service_type_support_t* t_supp_read_mreg       = ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, ReadMRegister);
    const rosidl_service_type_support_t* t_supp_write_mreg      = ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, WriteMRegister);

    ret = rclc_service_init_default(&g_serviceReadSingleIO  , &g_microRosNodeInfo.node, t_supp_read_single_io , SERVICE_NAME_READ_SINGLE_IO);
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_READ_SINGLE_IO, "Failed to init service (%d)", (int)ret);
    ret = rclc_service_init_default(&g_serviceReadGroupIO   , &g_microRosNodeInfo.node, t_supp_read_group_io  , SERVICE_NAME_READ_GROUP_IO);
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_READ_GROUP_IO, "Failed to init service (%d)", (int)ret);
    ret = rclc_service_init_default(&g_serviceWriteSingleIO , &g_microRosNodeInfo.node, t_supp_write_single_io, SERVICE_NAME_WRITE_SINGLE_IO);
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_WRITE_SINGLE_IO, "Failed to init service (%d)", (int)ret);
    ret = rclc_service_init_default(&g_serviceWriteGroupIO  , &g_microRosNodeInfo.node, t_supp_write_group_io , SERVICE_NAME_WRITE_GROUP_IO);
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_WRITE_GROUP_IO, "Failed to init service (%d)", (int)ret);
    ret = rclc_service_init_default(&g_serviceReadMRegister , &g_microRosNodeInfo.node, t_supp_read_mreg      , SERVICE_NAME_READ_MREGISTER);
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_READ_M_REG, "Failed to init service (%d)", (int)ret);
    ret = rclc_service_init_default(&g_serviceWriteMRegister, &g_microRosNodeInfo.node, t_supp_write_mreg     , SERVICE_NAME_WRITE_MREGISTER);
    motoRosAssert_withMsg(ret == RCL_RET_OK, SUBCODE_FAIL_INIT_SERVICE_WRITE_M_REG, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_ReadWriteIO.resp_single_io_read.message);
    rosidl_runtime_c__String__init(&g_messages_ReadWriteIO.resp_group_io_read.message);
    rosidl_runtime_c__String__init(&g_messages_ReadWriteIO.resp_single_io_write.message);
    rosidl_runtime_c__String__init(&g_messages_ReadWriteIO.resp_group_io_write.message);
    rosidl_runtime_c__String__init(&g_messages_ReadWriteIO.resp_mreg_read.message);
    rosidl_runtime_c__String__init(&g_messages_ReadWriteIO.resp_mreg_write.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_rw_io_init);
}

void Ros_ServiceReadWriteIO_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(svc_rw_io_fini);

    rcl_ret_t ret;

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_READ_SINGLE_IO "");
    ret = rcl_service_fini(&g_serviceReadSingleIO, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_READ_SINGLE_IO " service: %d", ret);

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_READ_GROUP_IO "");
    ret = rcl_service_fini(&g_serviceReadGroupIO, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_READ_GROUP_IO " service: %d", ret);

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_WRITE_SINGLE_IO "");
    ret = rcl_service_fini(&g_serviceWriteSingleIO, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_WRITE_SINGLE_IO " service: %d", ret);

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_WRITE_GROUP_IO "");
    ret = rcl_service_fini(&g_serviceWriteGroupIO, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_WRITE_GROUP_IO " service: %d", ret);

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_READ_MREGISTER "");
    ret = rcl_service_fini(&g_serviceReadMRegister, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_READ_MREGISTER " service: %d", ret);

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_WRITE_MREGISTER "");
    ret = rcl_service_fini(&g_serviceWriteMRegister, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg("Failed cleaning up " SERVICE_NAME_WRITE_MREGISTER " service: %d", ret);

    rosidl_runtime_c__String__fini(&g_messages_ReadWriteIO.resp_single_io_read.message);
    rosidl_runtime_c__String__fini(&g_messages_ReadWriteIO.resp_group_io_read.message);
    rosidl_runtime_c__String__fini(&g_messages_ReadWriteIO.resp_single_io_write.message);
    rosidl_runtime_c__String__fini(&g_messages_ReadWriteIO.resp_group_io_write.message);
    rosidl_runtime_c__String__fini(&g_messages_ReadWriteIO.resp_mreg_read.message);
    rosidl_runtime_c__String__fini(&g_messages_ReadWriteIO.resp_mreg_write.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_rw_io_fini);
}

void Ros_ServiceReadSingleIO_Trigger(const void* request_msg, void* response_msg)
{
    MP_IO_INFO ioReadInfo;
    USHORT ioValue = 0;

    motoros2_interfaces__srv__ReadSingleIO_Request* request =
        (motoros2_interfaces__srv__ReadSingleIO_Request*) request_msg;
    motoros2_interfaces__srv__ReadSingleIO_Response* response =
        (motoros2_interfaces__srv__ReadSingleIO_Response*) response_msg;

    if (Ros_IoServer_IsValidReadAddress(request->address, IO_ACCESS_BIT))
    {
        ioReadInfo.ulAddr = request->address;
        LONG apiRet = mpReadIO(&ioReadInfo, &ioValue, QUANTITY_BIT);

        response->value = ioValue;
        response->result_code = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_READ_API_ERROR;
        response->success = (apiRet == OK);
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
    else
    {
        response->value = 0;
        response->result_code = IO_RESULT_READ_ADDRESS_INVALID;
        response->success = FALSE;
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
}

void Ros_ServiceReadGroupIO_Trigger(const void* request_msg, void* response_msg)
{
    motoros2_interfaces__srv__ReadGroupIO_Request* request =
        (motoros2_interfaces__srv__ReadGroupIO_Request*) request_msg;
    motoros2_interfaces__srv__ReadGroupIO_Response* response =
        (motoros2_interfaces__srv__ReadGroupIO_Response*) response_msg;

    if (Ros_IoServer_IsValidReadAddress(request->address, IO_ACCESS_GROUP))
    {
        MP_IO_INFO ioReadInfo[8];
        USHORT ioValue[8];

        for (int i = 0; i < 8; i += 1)
        {
            ioReadInfo[i].ulAddr = (request->address * 10) + i;
        }
        LONG apiRet = mpReadIO(ioReadInfo, ioValue, QUANTITY_BYTE);

        int resultValue = 0;
        for (int i = 0; i < 8; i += 1)
        {
            resultValue |= (ioValue[i] << i);
        }

        response->value = resultValue;
        response->result_code = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_READ_API_ERROR;
        response->success = (apiRet == OK);
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
    else
    {
        response->value = 0;
        response->result_code = IO_RESULT_READ_ADDRESS_INVALID;
        response->success = FALSE;
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
}

void Ros_ServiceWriteSingleIO_Trigger(const void* request_msg, void* response_msg)
{
    BOOL bAddressOk;
    BOOL bValueOk;

    motoros2_interfaces__srv__WriteSingleIO_Request* request =
        (motoros2_interfaces__srv__WriteSingleIO_Request*) request_msg;
    motoros2_interfaces__srv__WriteSingleIO_Response* response =
        (motoros2_interfaces__srv__WriteSingleIO_Response*) response_msg;

    bAddressOk = Ros_IoServer_IsValidWriteAddress(request->address, IO_ACCESS_BIT);
    bValueOk = Ros_IoServer_IsValidWriteValue(request->value, IO_ACCESS_BIT);

    if (bAddressOk && bValueOk)
    {
        MP_IO_DATA ioWriteData;
        ioWriteData.ulAddr = request->address;
        ioWriteData.ulValue = request->value;
        LONG apiRet = mpWriteIO(&ioWriteData, QUANTITY_BIT);

        response->result_code = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_WRITE_API_ERROR;
        response->success = (apiRet == OK);
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
    else
    {
        if (!bAddressOk)
            response->result_code = IO_RESULT_WRITE_ADDRESS_INVALID;
        else if (!bValueOk)
            response->result_code = IO_RESULT_WRITE_VALUE_INVALID;

        response->success = FALSE;
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
}

void Ros_ServiceWriteGroupIO_Trigger(const void* request_msg, void* response_msg)
{
    BOOL bAddressOk;
    BOOL bValueOk;

    motoros2_interfaces__srv__WriteGroupIO_Request* request =
        (motoros2_interfaces__srv__WriteGroupIO_Request*) request_msg;
    motoros2_interfaces__srv__WriteGroupIO_Response* response =
        (motoros2_interfaces__srv__WriteGroupIO_Response*) response_msg;

    bAddressOk = Ros_IoServer_IsValidWriteAddress(request->address, IO_ACCESS_GROUP);
    bValueOk = Ros_IoServer_IsValidWriteValue(request->value, IO_ACCESS_GROUP);

    if (bAddressOk && bValueOk)
    {
        MP_IO_DATA ioWriteData[8];

        for (int i = 0; i < QUANTITY_BYTE; i += 1)
        {
            ioWriteData[i].ulAddr = (request->address * 10) + i;
            ioWriteData[i].ulValue = (request->value & (1 << i)) >> i;
        }
        LONG apiRet = mpWriteIO(ioWriteData, QUANTITY_BYTE);

        response->result_code = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_WRITE_API_ERROR;
        response->success = (apiRet == OK);
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
    else
    {
        if (!bAddressOk)
            response->result_code = IO_RESULT_WRITE_ADDRESS_INVALID;
        else if (!bValueOk)
            response->result_code = IO_RESULT_WRITE_VALUE_INVALID;

        response->success = FALSE;
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
}

void Ros_ServiceReadMRegister_Trigger(const void* request_msg, void* response_msg)
{
    MP_IO_INFO ioReadInfo;
    USHORT ioValue = 0;

    motoros2_interfaces__srv__ReadMRegister_Request* request =
        (motoros2_interfaces__srv__ReadMRegister_Request*) request_msg;
    motoros2_interfaces__srv__ReadMRegister_Response* response =
        (motoros2_interfaces__srv__ReadMRegister_Response*) response_msg;

    if (request->address < 1000000)
        request->address += 1000000;

    if (Ros_IoServer_IsValidReadAddress(request->address, IO_ACCESS_REGISTER))
    {
        ioReadInfo.ulAddr = request->address;
        LONG apiRet = mpReadIO(&ioReadInfo, &ioValue, QUANTITY_BIT);

        response->value = ioValue;
        response->success = (apiRet == OK);
        response->result_code = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_READ_API_ERROR;
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
    else
    {
        response->value = 0;
        response->success = FALSE;
        response->result_code = IO_RESULT_READ_ADDRESS_INVALID;
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }}

void Ros_ServiceWriteMRegister_Trigger(const void* request_msg, void* response_msg)
{
    MP_IO_DATA ioWriteData;
    BOOL bAddressOk;
    BOOL bValueOk;

    motoros2_interfaces__srv__WriteMRegister_Request* request =
        (motoros2_interfaces__srv__WriteMRegister_Request*) request_msg;
    motoros2_interfaces__srv__WriteMRegister_Response* response =
        (motoros2_interfaces__srv__WriteMRegister_Response*) response_msg;

    if (request->address < 1000000)
        request->address += 1000000;

    bAddressOk = Ros_IoServer_IsValidWriteAddress(request->address, IO_ACCESS_REGISTER);
    bValueOk = Ros_IoServer_IsValidWriteValue(request->value, IO_ACCESS_REGISTER);

    if (bAddressOk && bValueOk)
    {
        ioWriteData.ulAddr = request->address;
        ioWriteData.ulValue = request->value;
        LONG apiRet = mpWriteIO(&ioWriteData, QUANTITY_BIT);

        response->result_code = (apiRet == OK) ? IO_RESULT_OK : IO_RESULT_WRITE_API_ERROR;
        response->success = (apiRet == OK);
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
    else
    {
        if (!bAddressOk)
            response->result_code = IO_RESULT_WRITE_ADDRESS_INVALID;
        else if (!bValueOk)
            response->result_code = IO_RESULT_WRITE_VALUE_INVALID;

        response->success = FALSE;
        rosidl_runtime_c__String__assign(&response->message,
            Ros_IoServer_ResultCodeToStr(response->result_code));
    }
}

BOOL Ros_IoServer_IsValidReadAddress(UINT32 address, IoAccessSize size)
{
    int mod = 0;

    if (size == IO_ACCESS_GROUP)
        address *= 10;

    mod = address % 10;

    //last digit cannot end in 8 or 9, unless it is an M Register
    if (size != IO_ACCESS_REGISTER && mod > 7)
        return FALSE;

    if ((address >= GENERALINMIN && address <= GENERALINMAX) ||
        (address >= GENERALOUTMIN && address <= GENERALOUTMAX) ||
        (address >= EXTERNALINMIN && address <= EXTERNALINMAX) ||
        (address >= NETWORKINMIN && address <= NETWORKINMAX) ||
        (address >= NETWORKOUTMIN && address <= NETWORKOUTMAX) ||
        (address >= EXTERNALOUTMIN && address <= EXTERNALOUTMAX) ||
        (address >= SPECIFICINMIN && address <= SPECIFICINMAX) ||
        (address >= SPECIFICOUTMIN && address <= SPECIFICOUTMAX) ||
        (address >= IFPANELMIN && address <= IFPANELMAX) ||
        (address >= AUXRELAYMIN && address <= AUXRELAYMAX) ||
        (address >= CONTROLSTATUSMIN && address <= CONTROLSTATUSMAX) ||
        (address >= PSEUDOINPUTMIN && address <= PSEUDOINPUTMAX) ||
        (address >= REGISTERMIN && address <= REGISTERMAX_READ))
    {
        return TRUE;
    }
    else
        return FALSE;
}

BOOL Ros_IoServer_IsValidWriteAddress(UINT32 address, IoAccessSize size)
{
    int mod = 0;

    if (size == IO_ACCESS_GROUP)
        address *= 10;

    mod = address % 10;

    //last digit cannot end in 8 or 9, unless it is an M Register
    if (size != IO_ACCESS_REGISTER && mod > 7)
        return FALSE;

    if ((address >= GENERALOUTMIN && address <= GENERALOUTMAX) ||
        (address >= NETWORKINMIN && address <= NETWORKINMAX) ||
        (address >= IFPANELMIN && address <= IFPANELMAX) ||
        (address >= REGISTERMIN && address <= REGISTERMAX_WRITE))
    {
        return TRUE;
    }
    else
        return FALSE;
}

BOOL Ros_IoServer_IsValidWriteValue(UINT32 value, IoAccessSize size)
{
    if (size == IO_ACCESS_REGISTER && value > 0xFFFF)
        return FALSE;

    if (size == IO_ACCESS_GROUP && value > 0xFF)
        return FALSE;

    if (size == IO_ACCESS_BIT && value > 1)
        return FALSE;

    return TRUE;
}

const char* const Ros_IoServer_ResultCodeToStr(UINT32 resultCode)
{
    //message strings defined in motoros2_interfaces/msg/IoResultCodes.msg
    switch(resultCode)
    {
        case IO_RESULT_OK:
            return motoros2_interfaces__msg__IoResultCodes__OK_STR;
        case IO_RESULT_READ_ADDRESS_INVALID:
            return motoros2_interfaces__msg__IoResultCodes__READ_ADDRESS_INVALID_STR;
        case IO_RESULT_WRITE_ADDRESS_INVALID:
            return motoros2_interfaces__msg__IoResultCodes__WRITE_ADDRESS_INVALID_STR;
        case IO_RESULT_WRITE_VALUE_INVALID:
            return motoros2_interfaces__msg__IoResultCodes__WRITE_VALUE_INVALID_STR;
        case IO_RESULT_READ_API_ERROR:
            return motoros2_interfaces__msg__IoResultCodes__READ_API_ERROR_STR;
        case IO_RESULT_WRITE_API_ERROR:
            return motoros2_interfaces__msg__IoResultCodes__WRITE_API_ERROR_STR;
        default:
            return motoros2_interfaces__msg__IoResultCodes__UNKNOWN_API_ERROR_STR;
    }
}
