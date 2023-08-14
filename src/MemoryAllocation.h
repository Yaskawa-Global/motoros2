// MemoryAllocation.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_MEMORY_ALLOCATION_H
#define MOTOROS2_MEMORY_ALLOCATION_H

//------------------------------------
//Dynamic Memory
//------------------------------------
extern void Ros_Allocation_Initialize(rcl_allocator_t* const allocator);

extern rcl_allocator_t g_motoros2_Allocator;

#endif  // MOTOROS2_MEMORY_ALLOCATION_H
