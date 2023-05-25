// MemoryAllocation.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

rcl_allocator_t g_motoros2_Allocator;

void* __motoplus_allocate(size_t size, void* state)
{
    RCUTILS_CAN_RETURN_WITH_ERROR_OF(NULL);

    RCUTILS_UNUSED(state);
    void* newPtr = mpMalloc(size);
    if (!newPtr)
        mpSetAlarm(ALARM_ALLOCATION_FAIL, APPLICATION_NAME " OUT OF MEMORY", SUBCODE_ALLOCATION_MALLOC);
    return newPtr;
}

void __motoplus_deallocate(void* pointer, void* state)
{
    RCUTILS_UNUSED(state);
    mpFree(pointer);
}

void* __motoplus_reallocate(void* pointer, size_t size, void* state)
{
    RCUTILS_CAN_RETURN_WITH_ERROR_OF(NULL);
    RCUTILS_UNUSED(state);

    void* newPtr = mpMalloc(size);
    if (!newPtr)
    {
        mpSetAlarm(ALARM_ALLOCATION_FAIL, APPLICATION_NAME " OUT OF MEMORY", SUBCODE_ALLOCATION_REALLOC);
        return NULL;
    }

    // according to POSIX, pointer == null -> malloc(..)
    if (pointer)
    {
        memcpy(newPtr, pointer, size);
        mpFree(pointer);
    }

    return newPtr;
}

void* __motoplus_zero_allocate(size_t number_of_elements, size_t size_of_element, void* state)
{
    RCUTILS_CAN_RETURN_WITH_ERROR_OF(NULL);
    RCUTILS_UNUSED(state);

    if ((number_of_elements == 0) || (size_of_element == 0))
    {
        return NULL;
    }
    size_t size = size_of_element * number_of_elements;

    void* newPtr = mpMalloc(size);
    if (!newPtr)
        mpSetAlarm(ALARM_ALLOCATION_FAIL, APPLICATION_NAME " OUT OF MEMORY", SUBCODE_ALLOCATION_CALLOC);
    if (newPtr)
        bzero(newPtr, size);

    return newPtr;
}

void Ros_Allocation_Initialize()
{
    //initialize memory allocation for microros
    g_motoros2_Allocator.allocate = __motoplus_allocate;
    g_motoros2_Allocator.deallocate = __motoplus_deallocate;
    g_motoros2_Allocator.reallocate = __motoplus_reallocate;
    g_motoros2_Allocator.zero_allocate = __motoplus_zero_allocate;
    g_motoros2_Allocator.state = NULL;

    //configure Micro-ROS to use our allocator. Errors are fatal.
    motoRosAssert(rcutils_set_default_allocator(&g_motoros2_Allocator), SUBCODE_FAIL_MEM_ALLOC_CFG);
}
