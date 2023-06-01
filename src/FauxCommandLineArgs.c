// FauxCommandLineArgs.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"
#include <string.h>
#include <rcutils/strdup.h>


//TODO: Once https://github.com/ros2/rcl/issues/998 is addressed, this would no longer
//      be necessary. We could construct rcl_remap_t instances directly without having
//      to construct a fake argv and go through rcl_parse_arguments(..).
//NOTE: this is similar to what ComponentManager::create_node_options(..) does.
//
// Assumptions:
//
//  1. 'remap_rules_str' points to a space-separated string of 'from:=to' remap rules
//  2. caller owns 'remap_rules_str'
//  3. 'remap_rules_str' may be clobbered by this function
//  4. 'out_array' is an array of 'char*' of length 'out_array_len'
//  5. all elements of 'out_array' are NULL pointers
//  6. caller owns 'out_array'
//  7. caller will clean up 'out_array' (ie: all memory allocated by this function)
//  8. 'remap_rules_str' does not contain more than MAX_REMAP_RULE_NUM (16) remap rules
//  9. 'remap_rules_str' is no longer than MAX_REMAP_RULE_LEN (255 chars)
// 10. 'out_array_len' represents the nr of slots in 'out_array'
//
// Return value:
//
//  <  0: error
//       -1: output array too small
//  >= 2: number of elements in the fake argv array
//
int Ros_ConstructFauxArgv(char* const remap_rules_str, char* out_array[], size_t out_array_len)
{
    // ros args begin and end flags, and disable-stdout-logs arg
    const size_t MIN_ARGVS = 1 + 1 + 1;
    const size_t MAX_ARGVS = (MAX_REMAP_RULE_NUM * 2) + MIN_ARGVS;

    if (out_array_len < MIN_ARGVS)
        return -1;

    size_t num_remap_rules_ = 0;
    size_t num_argvs_ = 0;

    // add ROS CLA prefix
    out_array[num_argvs_++] = rcutils_strdup(RCL_ROS_ARGS_FLAG, g_motoros2_Allocator);

    rcutils_string_array_t rules = rcutils_get_zero_initialized_string_array();
    rcutils_split(remap_rules_str, ' ', g_motoros2_Allocator, &rules);
    for (int i = 0; i < rules.size && num_argvs_ < out_array_len; i += 1)
    {
        // ros arg indicator
        out_array[num_argvs_++] = rcutils_strdup(RCL_SHORT_REMAP_FLAG, g_motoros2_Allocator);
        // store ptr to rule
        out_array[num_argvs_++] = rcutils_strdup(rules.data[i], g_motoros2_Allocator);

        Ros_Debug_BroadcastMsg("remap rule(%d): %s '%s'", num_remap_rules_,
            out_array[num_argvs_-2], out_array[num_argvs_-1]);

        num_remap_rules_++;

        if (num_argvs_ >= MAX_ARGVS)
        {
            mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Too many remap rules", SUBCODE_CONFIGURATION_TOO_MANY_REMAP_RULES1);
            return -1;
        }

        if (num_remap_rules_ >= MAX_REMAP_RULE_NUM)
        {
            mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Too many remap rules", SUBCODE_CONFIGURATION_TOO_MANY_REMAP_RULES2);
            return -1;
        }
    }

    int ret = rcutils_string_array_fini(&rules);
    //ignored
    RCUTILS_UNUSED(ret);

    if (num_argvs_ == out_array_len)
        return -1;

    if ((num_argvs_ - 1) % 2 != 0)
    {
        mpSetAlarm(ALARM_CONFIGURATION_FAIL, "Invalid remap rule format", SUBCODE_CONFIGURATION_INVALID_REMAP_RULE_FORMAT);
        return -1;
    }

    // disable logging to stdout: it will end up on the console of the
    // controller which is invisible to users
    out_array[num_argvs_++] = rcutils_strdup(
        "--disable-stdout-logs", g_motoros2_Allocator);

    // add ROS CLA suffix
    out_array[num_argvs_++] = rcutils_strdup(
        RCL_ROS_ARGS_EXPLICIT_END_TOKEN, g_motoros2_Allocator);

    Ros_Debug_BroadcastMsg("num parsed remap rules: %d", num_remap_rules_);
    Ros_Debug_BroadcastMsg("faux argc: %d", num_argvs_);

    // done
    return num_argvs_;
}

void Ros_CleanupFauxArgv(char** arr, size_t arr_len)
{
    for (size_t i = 0; i < arr_len; ++i)
    {
        if (arr[i])
            g_motoros2_Allocator.deallocate(arr[i], g_motoros2_Allocator.state);
    }
}
