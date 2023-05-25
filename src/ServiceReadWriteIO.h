//ServiceReadWriteIO.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_READ_WRITE_IO_H
#define MOTOROS2_SERVICE_READ_WRITE_IO_H

extern rcl_service_t g_serviceReadSingleIO;
extern rcl_service_t g_serviceReadGroupIO;
extern rcl_service_t g_serviceWriteSingleIO;
extern rcl_service_t g_serviceWriteGroupIO;
extern rcl_service_t g_serviceReadMRegister;
extern rcl_service_t g_serviceWriteMRegister;

typedef struct
{
    motoros2_interfaces__srv__ReadSingleIO_Request req_single_io_read;
    motoros2_interfaces__srv__ReadSingleIO_Response resp_single_io_read;

    motoros2_interfaces__srv__ReadGroupIO_Request req_group_io_read;
    motoros2_interfaces__srv__ReadGroupIO_Response resp_group_io_read;

    motoros2_interfaces__srv__WriteSingleIO_Request req_single_io_write;
    motoros2_interfaces__srv__WriteSingleIO_Response resp_single_io_write;

    motoros2_interfaces__srv__WriteGroupIO_Request req_group_io_write;
    motoros2_interfaces__srv__WriteGroupIO_Response resp_group_io_write;

    motoros2_interfaces__srv__ReadMRegister_Request req_mreg_read;
    motoros2_interfaces__srv__ReadMRegister_Response resp_mreg_read;

    motoros2_interfaces__srv__WriteMRegister_Request req_mreg_write;
    motoros2_interfaces__srv__WriteMRegister_Response resp_mreg_write;
} ServiceReadWriteIO_Messages;

extern ServiceReadWriteIO_Messages g_messages_ReadWriteIO;

void Ros_ServiceReadWriteIO_Initialize();
void Ros_ServiceReadWriteIO_Cleanup();

void Ros_ServiceReadSingleIO_Trigger(const void* request_msg, void* response_msg);
void Ros_ServiceReadGroupIO_Trigger(const void* request_msg, void* response_msg);
void Ros_ServiceWriteSingleIO_Trigger(const void* request_msg, void* response_msg);
void Ros_ServiceWriteGroupIO_Trigger(const void* request_msg, void* response_msg);
void Ros_ServiceReadMRegister_Trigger(const void* request_msg, void* response_msg);
void Ros_ServiceWriteMRegister_Trigger(const void* request_msg, void* response_msg);


#endif // MOTOROS2_SERVICE_READ_WRITE_IO_H
