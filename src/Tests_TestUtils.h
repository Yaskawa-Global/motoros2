// Tests_TestUtils.h

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_TESTS_TEST_UTILS_H
#define MOTOROS2_TESTS_TEST_UTILS_H

#ifdef MOTOROS2_TESTING_ENABLE

extern BOOL Ros_Testing_CompareDouble(double a, double b);
extern BOOL Ros_Testing_CompareLong(long a, long b);
extern BOOL Ros_Testing_INT64_Equals(INT64 a, INT64 b);
extern BOOL Ros_Testing_Timespec_Equals(const struct timespec* lhs, const struct timespec* rhs);
#endif //MOTOROS2_TESTING_ENABLE

#endif  // MOTOROS2_TESTS_TEST_UTILS_H
