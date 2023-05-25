// MemoryTracing.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_MEMORY_TRACING_H
#define MOTOROS2_MEMORY_TRACING_H


#ifdef MOTOROS2_MEM_TRACE_ENABLE


// NOTE: cannot wrap this in a do '{} while(0)' as otherwise the variables
//       would be declared in that scope -- and cannot be referenced later
#define MOTOROS2_MEM_TRACE_START(name) \
    uint64_t __mem_trace_point_start_##name = mpNumBytesFree();


#define MOTOROS2_MEM_TRACE_MEM_FREE(name) do                     \
{                                                                \
    Ros_Debug_BroadcastMsg("%s(..):%s: memory free: %llu bytes", \
        __func__, #name, (uint64_t) mpNumBytesFree());           \
} while(0);


#define MOTOROS2_MEM_TRACE_REPORT(name) do                                     \
{                                                                              \
    uint64_t __mem_trace_point_end_##name = mpNumBytesFree();                  \
    /* overflow possible, but unlikely given the max mem of YRC1000/DX200 */   \
    int64_t __mem_trace_point_delta_##name =                                   \
        __mem_trace_point_end_##name - __mem_trace_point_start_##name;         \
    Ros_Debug_BroadcastMsg("%s(..):%s: memory free: %llu bytes, delta: %+lld", \
        __func__, #name,                                                       \
        __mem_trace_point_end_##name, __mem_trace_point_delta_##name);         \
} while(0);


#else  // MOTOROS2_MEM_TRACE_ENABLE


// no-ops if disabled
#define MOTOROS2_MEM_TRACE_START(name)
#define MOTOROS2_MEM_TRACE_MEM_FREE(name)
#define MOTOROS2_MEM_TRACE_REPORT(name)


#endif  // MOTOROS2_MEM_TRACE_ENABLE


#endif  // MOTOROS2_MEMORY_TRACING_H
