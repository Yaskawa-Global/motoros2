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
// Ros_MotionControl_StopMotion applies and then releases command HOLD on a
// successful stop. Pendant and external HOLD are separate controller HOLD
// sources. If stopping or queue cleanup fails, command HOLD and the software
// stop remain asserted; recovery requires resolving the fault before motion can
// resume.
//
// Request: empty - the abort is unconditional and takes no arguments.
// Response: 'success' (BOOL) and 'message' (human-readable detail, always
// populated). 'success' is FALSE when the stop could not be CONFIRMED (message
// processing did not quiesce, or an increment queue was locked); the motion mode
// is exited either way, so a FALSE reply means "disarmed, but motion may not have
// stopped - escalate to the E-stop". See the ORDERING note in
// ServiceAbortPointQueue.c.
//
// The integrator-facing copy of all of the above lives in AbortPointQueue.srv
// (rendered by 'ros2 interface show') and in doc/ros_api.md; keep the three in
// step when changing behaviour.
//
// Relationship to 'stop_traj_mode': stop_traj_mode is the graceful exit and
// REFUSES while the increment queue still holds work (see ServiceStopTrajMode.c)
// - precisely the state an abort has to deal with. This service performs the
// same stop sequence WITHOUT that refusal, so it exits point-queue mode. Calls
// outside point-queue mode are rejected without affecting FollowJointTrajectory.
// It is the escalation path when a graceful point-queue stop cannot be honoured.


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
