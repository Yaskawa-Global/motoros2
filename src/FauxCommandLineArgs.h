// FauxCommandLineArgs.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_FAUX_COMMAND_LINE_ARGS_H
#define MOTOROS2_FAUX_COMMAND_LINE_ARGS_H

extern int Ros_ConstructFauxArgv(char* const remap_rules_str, char* out_array[], size_t out_array_len);
extern void Ros_CleanupFauxArgv(char** arr, size_t arr_len);

#endif  // MOTOROS2_FAUX_COMMAND_LINE_ARGS_H
