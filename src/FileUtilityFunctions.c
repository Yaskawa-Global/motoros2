//FileUtilityFunctions.c

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#include "MotoROS.h"

BOOL FileUtilityFunctions_ReadLine(int fd, char* buffer, int len)
{
    int bytesRead;
    char* mid;

    //read more than one line
    bzero(buffer, len);
    bytesRead = mpRead(fd, buffer, len);

    if (bytesRead == -1 || bytesRead == 0)
        return false;

    //find eol
    mid = strstr(buffer, "\r\n");

    //eof
    if (mid == NULL)
        return true;

    //move file pointer back to eol
    int backup = ((bytesRead - (mid - buffer)) * -1) + 2;
    mpLseek(fd, backup, SEEK_CUR);

    *mid = '\0';

    return true;
}

BOOL FileUtilityFunctions_WriteLine(int fd, const char* fmt, ...)
{
    const int MAX_LINE_LEN = 512;
    va_list va;
    char buffer[MAX_LINE_LEN];

    va_start(va, fmt);
    vsnprintf(buffer, MAX_LINE_LEN, fmt, va);
    va_end(va);

    int bytesWrote;

    strcat(buffer, "\r\n");

    bytesWrote = mpWrite(fd, buffer, Ros_strnlen(buffer, MAX_LINE_LEN));

    if (bytesWrote == -1 || bytesWrote == 0)
        return false;

    return true;
}
