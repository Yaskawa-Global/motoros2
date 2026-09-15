//ServiceAbortPointQueue.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_SERVICE_ABORT_POINT_QUEUE_H
#define MOTOROS2_SERVICE_ABORT_POINT_QUEUE_H

// Service 'abort_point_queue': halt motion NOW and discard every point the
// controller has already accepted, whatever the queue currently holds.
//
// >>> SOFTWARE STOP ONLY - NOT A SAFETY-RATED EMERGENCY STOP <<<
// This service is as fast as software can stop the arm, and no faster. It is
// subject to controller scheduling, message-processing latency and the servo
// deceleration ramp, and it does nothing at all if MotoROS2 or the agent link
// is unhealthy. It is NOT an emergency stop and MUST NOT be presented to an
// operator as one: only the physical safety circuit (E-stop button, safety
// fence, PFL) provides emergency-stop assurance. Integrators must keep the
// hardware E-stop as the sole protective stop.
//
// Request: empty.
// Response: 'success' (BOOL) and 'message' (human-readable detail). Note that
// AbortPointQueue.srv deliberately carries no comments (its request has no
// fields), so THIS header is the normative documentation of the service.
//
// Relationship to 'stop_traj_mode': stop_traj_mode is the graceful exit and
// REFUSES while the increment queue still holds work (see ServiceStopTrajMode.c)
// - precisely the state an abort has to deal with. This service performs the
// same stop sequence WITHOUT that refusal, so it always exits point-queue /
// trajectory mode. It is therefore the escalation path when a graceful stop
// cannot be honoured, and it is expected to discard queued motion.


extern rcl_service_t g_serviceAbortPointQueue;

typedef struct
{
    motoros2_interfaces__srv__AbortPointQueue_Request request;
    motoros2_interfaces__srv__AbortPointQueue_Response response;
} ServiceAbortPointQueue_Messages;
extern ServiceAbortPointQueue_Messages g_messages_AbortPointQueue;

extern void Ros_ServiceAbortPointQueue_Initialize();
extern void Ros_ServiceAbortPointQueue_Cleanup();

extern void Ros_ServiceAbortPointQueue_Trigger(const void* request_msg, void* response_msg);


#endif  // MOTOROS2_SERVICE_ABORT_POINT_QUEUE_H
