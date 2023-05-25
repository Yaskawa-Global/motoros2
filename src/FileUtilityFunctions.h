//FileUtilityFunctions.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_FILE_UTILITY_FUNCTIONS_H
#define MOTOROS2_FILE_UTILITY_FUNCTIONS_H

extern BOOL FileUtilityFunctions_ReadLine(int fd, char* buffer, int len);
extern BOOL FileUtilityFunctions_WriteLine(int fd, const char* fmt, ...);

#endif
