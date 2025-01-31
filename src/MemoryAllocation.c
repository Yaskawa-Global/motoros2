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

void Ros_Allocation_Initialize(rcl_allocator_t* const allocator)
{
    //initialize memory allocation for microros
    allocator->allocate = __motoplus_allocate;
    allocator->deallocate = __motoplus_deallocate;
    allocator->reallocate = __motoplus_reallocate;
    allocator->zero_allocate = __motoplus_zero_allocate;
    allocator->state = NULL;

    //configure Micro-ROS to use our allocator. Errors are fatal.
    motoRos_ASSERT_TRUE(rcutils_set_default_allocator(allocator), SUBCODE_FAIL_MEM_ALLOC_CFG);
}
