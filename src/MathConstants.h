//MathConstants.h

// SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_MATH_CONSTANTS_H
#define MOTOROS2_MATH_CONSTANTS_H


// deg->rad (by multiplication)
#define RAD_PER_DEGREE (0.01745329251994)

// rad->deg (by multiplication)
#define DEGREES_PER_RAD (57.295779513082)

// macro
#define NANOMETERS_TO_METERS(x)    (x * 0.000001)

// used for comparing floating point values
#define EPSILON_TOLERANCE_DOUBLE (0.001)

// maximum pulse count deviation for test calculations
#define EPSILON_TOLERANCE_PULSECOUNT (3)


#endif // MOTOROS2_MATH_CONSTANTS_H
