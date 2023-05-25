//InformCheckerAndGenerator.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_INFORM_CHECKER_AND_GENERATOR_H
#define MOTOROS2_INFORM_CHECKER_AND_GENERATOR_H

// Verify that the INFORM job is properly formatted. If not,
// then build the job automatically
extern void Ros_InformChecker_ValidateJob();

#endif
