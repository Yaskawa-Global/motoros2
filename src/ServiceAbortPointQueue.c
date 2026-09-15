//ServiceAbortPointQueue.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_service_t g_serviceAbortPointQueue;

ServiceAbortPointQueue_Messages g_messages_AbortPointQueue;

// shorten the typename a little, locally
typedef motoros2_interfaces__srv__AbortPointQueue_Response AbortPointQueue_Response;

void Ros_ServiceAbortPointQueue_Initialize()
{
    MOTOROS2_MEM_TRACE_START(svc_abort_point_queue_init);

    rcl_ret_t ret = rclc_service_init_default(&g_serviceAbortPointQueue, &g_microRosNodeInfo.node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(motoros2_interfaces, srv, AbortPointQueue),
        SERVICE_NAME_ABORT_POINT_QUEUE);
    motoRos_RCLAssertOK_withMsg(ret, SUBCODE_FAIL_INIT_SERVICE_ABORT_POINT_QUEUE, "Failed to init service (%d)", (int)ret);

    rosidl_runtime_c__String__init(&g_messages_AbortPointQueue.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_abort_point_queue_init);
}

void Ros_ServiceAbortPointQueue_Cleanup()
{
    MOTOROS2_MEM_TRACE_START(svc_abort_point_queue_fini);

    rcl_ret_t ret;

    Ros_Debug_BroadcastMsg("Cleanup service " SERVICE_NAME_ABORT_POINT_QUEUE);
    ret = rcl_service_fini(&g_serviceAbortPointQueue, &g_microRosNodeInfo.node);
    if (ret != RCL_RET_OK)
        Ros_Debug_BroadcastMsg(
            "Failed cleaning up " SERVICE_NAME_ABORT_POINT_QUEUE " service: %d", ret);
    rosidl_runtime_c__String__fini(&g_messages_AbortPointQueue.response.message);

    MOTOROS2_MEM_TRACE_REPORT(svc_abort_point_queue_fini);
}

// Stop motion NOW and discard every accepted point, unconditionally.
//
// >>> SOFTWARE STOP ONLY - NOT A SAFETY-RATED EMERGENCY STOP <<<
// See the safety note in ServiceAbortPointQueue.h: only the physical safety
// circuit provides emergency-stop assurance.
//
// This service diverges from Ros_ServiceStopTrajMode_Trigger in exactly TWO
// places, both of them the point of the service, and both called out where they
// happen below:
//   1. it does not refuse when the increment queue is non-empty
//      (ServiceStopTrajMode.c) - that is exactly the state an abort exists to
//      resolve;
//   2. it exits the motion mode on the failure path too, instead of returning
//      early - see the ORDERING note in the body.
// Everything else deliberately reuses the existing stop machinery rather than
// adding a second stop path:
//
//   Ros_MotionControl_StopMotion(FALSE) raises bStopMotion, holds INIT_ROS so
//   the arm decelerates smoothly, WAITS for message processing to quiesce, then
//   clears the increment queues and (because the consumer is provably stopped)
//   flushes the accepted point rings directly - so the rings are empty on
//   return. This service therefore must NOT touch the lock-free rings itself;
//   doing so from a service context would race the interpolation task. See the
//   QUIESCED-CALLER-ONLY note on the ring's direct flush primitive in
//   PointQueue.{c,h}: StopMotion is the one caller allowed to use it.
void Ros_ServiceAbortPointQueue_Trigger(const void* request_msg, void* response_msg)
{
    RCL_UNUSED(request_msg);
    AbortPointQueue_Response* response = (AbortPointQueue_Response*)response_msg;

    if (!Ros_MotionControl_IsMotionMode_PointQueue())
    {
        response->success = FALSE;
        rosidl_runtime_c__String__assign(&response->message,
            "abort rejected: point-queue mode is not active");
        Ros_Debug_BroadcastMsg("%s: %s", __func__, response->message.data);
        return;
    }

    Ros_Debug_BroadcastMsg("%s: aborting point queue (software stop, NOT an E-stop)", __func__);

    BOOL bMotionStopped = Ros_MotionControl_StopMotion(FALSE);

    // ORDERING - DELIBERATE DIVERGENCE FROM ServiceStopTrajMode.c:
    // exit the motion mode and raise INCMOVE_DONE on BOTH paths, BEFORE looking
    // at the result. stop_traj_mode returns early when StopMotion fails, which is
    // defensible for a graceful request; for an escalation stop it produces the
    // worst possible end state, so this service does NOT copy it.
    //
    // StopMotion returns FALSE when the quiesce wait expires, a queue cannot be
    // cleared, or a HOLD operation fails. It retains bStopMotion and command HOLD
    // on an incomplete stop. Exiting point-queue mode is still required to reject
    // future admission and force explicit reinitialization. It is
    // a pure state change (StopTrajMode only clears the mode flags and drops
    // WAITING_ROS) that cannot fail and cannot make matters worse.
    //
    // Having disabled admission, report failure honestly rather than claiming a
    // stop we did not achieve; the caller must escalate to the physical E-stop.
    Ros_MotionControl_StopTrajMode();
    Ros_ServiceQueueTrajPointStream_ResetSequence();

    if (!bMotionStopped)
    {
        response->success = FALSE;
        rosidl_runtime_c__String__assign(&response->message,
            "abort INCOMPLETE: motion may not have stopped (message processing did not quiesce, "
            "or an increment queue/HOLD operation failed). Motion mode was exited, and the "
            "software stop remains latched. Treat the controller as unsafe and use the E-stop");

        Ros_Debug_BroadcastMsg("%s: %s", __func__, response->message.data);
        return;
    }

    Ros_Controller_SetIOState(IO_FEEDBACK_MP_INCMOVE_DONE, TRUE);

    response->success = TRUE;
    rosidl_runtime_c__String__assign(&response->message, "point queue aborted and motion mode exited");

    Ros_Debug_BroadcastMsg("%s: %s", __func__, response->message.data);
}
