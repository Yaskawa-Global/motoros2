//Quaternion_Conversion.h

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
// Copyright © 2010 - 2020 three.js authors
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


#ifndef MOTOROS2_QUATERNION_CONVERSION_H
#define MOTOROS2_QUATERNION_CONVERSION_H

typedef geometry_msgs__msg__Quaternion Quaternion;

/**
 * Convert Euler angles in degrees (ZYX-intrinsic) to their quaternion representation.
 *
 * NOTE: Euler angles are expected to be in one-tenth milli-degrees (ie: 0.0001 deg),
 *       such as are used for MP_COORD orientation fields.
 */
extern void QuatConversion_MpCoordOrient_To_GeomMsgsQuaternion(LONG rx_deg, LONG ry_deg, LONG rz_deg, Quaternion* q);

/**
 * Convert the quaternion q to its Euler angle (ZYX-intrinsic) representation.
 *
 * NOTE: Euler angles will be in one-tenth milli-degrees (ie: 0.0001 deg), such
 *       as are used for MP_COORD orientation fields.
 */
extern void QuatConversion_GeomMsgsQuaternion_To_MpCoordOrient(const Quaternion* const q, LONG* rx_deg, LONG* ry_deg, LONG* rz_deg);

#endif  // MOTOROS2_QUATERNION_CONVERSION_H
