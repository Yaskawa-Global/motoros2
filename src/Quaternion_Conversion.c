// =============================================================================
//
// QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(..) adapted from:
//
// SPDX-FileCopyrightText: 2011, John Fuller
// SPDX-License-Identifier: BSD-2-Clause
//
// Originally from:
//
//  https://www.mathworks.com/matlabcentral/fileexchange/20696-function-to-convert-between-dcm-euler-angles-quaternions-and-euler-vectors
//
// Original license:
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met :
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditionsand the following disclaimer in the documentation
//   and/or other materials provided with the distribution
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// =============================================================================
//
// QuatConversion_GeomMsgsQuaternion_To_MpCoordOrient(..) adapted from:
//
// SPDX-FileCopyrightText: 2010 - 2020 three.js authors
// SPDX-License-Identifier: MIT
//
// Originally from:
//
//  https://github.com/mrdoob/three.js
//
// Original license:
//
// The MIT License
//
// Copyright (c) 2010 - 2020 three.js authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this softwareand associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright noticeand this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// =============================================================================


#include "MotoROS.h"

#ifndef max
#define max(x, y)	(((x) < (y)) ? (y) : (x))
#endif

#ifndef min
#define min(x, y)	(((x) < (y)) ? (x) : (y))
#endif

double QuatConversion_clamp(double value, double _min, double _max)
{
    return max(_min, min(_max, value));
}

void QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(LONG rx_deg, LONG ry_deg, LONG rz_deg, Quaternion* q)
{
    // convert Euler angles to radians, and already assign to the quaternion's
    // fields to avoid using (more) temporary storage
    //
    // NOTE: assumption: Euler angles are in 0.1 milli-degrees (as is the case
    //       for MP_COORD orientation fields)
    q->x = rx_deg * RAD_PER_DEGREE * .0001;
    q->y = ry_deg * RAD_PER_DEGREE * .0001;
    q->z = rz_deg * RAD_PER_DEGREE * .0001;

    const double c1 = cos(q->x / 2);
    const double c2 = cos(q->y / 2);
    const double c3 = cos(q->z / 2);

    const double s1 = sin(q->x / 2);
    const double s2 = sin(q->y / 2);
    const double s3 = sin(q->z / 2);

    q->x = s1 * c2 * c3 - c1 * s2 * s3;
    q->y = c1 * s2 * c3 + s1 * c2 * s3;
    q->z = c1 * c2 * s3 - s1 * s2 * c3;
    q->w = c1 * c2 * c3 + s1 * s2 * s3;
}

void QuatConversion_GeomMsgsQuaternion_To_MpCoordOrient(const Quaternion* const q, LONG* rx_deg, LONG* ry_deg, LONG* rz_deg)
{
    const double x = q->x, y = q->y, z = q->z, w = q->w;
    const double x2 = x + x, y2 = y + y, z2 = z + z;
    const double xx = x * x2, xy = x * y2, xz = x * z2;
    const double yy = y * y2, yz = y * z2, zz = z * z2;
    const double wx = w * x2, wy = w * y2, wz = w * z2;

    const double sx = 1.0, sy = 1.0, sz = 1.0;

    double elements[16];

    elements[0] = (1 - (yy + zz)) * sx;
    elements[1] = (xy + wz) * sx;
    elements[2] = (xz - wy) * sx;
    elements[3] = 0;

    elements[4] = (xy - wz) * sy;
    elements[5] = (1 - (xx + zz)) * sy;
    elements[6] = (yz + wx) * sy;
    elements[7] = 0;

    elements[8] = (xz + wy) * sz;
    elements[9] = (yz - wx) * sz;
    elements[10] = (1 - (xx + yy)) * sz;
    elements[11] = 0;

    elements[12] = 0;
    elements[13] = 0;
    elements[14] = 0;
    elements[15] = 1;

    double m11 = elements[0], m12 = elements[4], m13 = elements[8];
    double m21 = elements[1], m22 = elements[5], m23 = elements[9];
    double m31 = elements[2], m32 = elements[6], m33 = elements[10];

    // these are unused
    // TODO(gavanderhoorn): decide whether to remove these instead
    (void) m13;
    (void) m23;

    // convert radians to Euler anles
    //
    // NOTE: assumption: Euler angles should be in 0.1 milli-degrees (as those
    //       are used for MP_COORD orientation fields)
    *ry_deg = (LONG)(asin(-QuatConversion_clamp(m31, -1, 1)) * DEGREES_PER_RAD * 10000);

    if (abs(m31) < 0.9999999)
    {
        *rx_deg = (LONG)(atan2(m32, m33) * DEGREES_PER_RAD * 10000);
        *rz_deg = (LONG)(atan2(m21, m11) * DEGREES_PER_RAD * 10000);
    }
    else
    {
        *rx_deg = 0;
        *rz_deg = (LONG)(atan2(-m12, m22) * DEGREES_PER_RAD * 10000);
    }
}
