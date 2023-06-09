//MotoPlusExterns.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_MOTOPLUS_EXTERNS_H
#define MOTOROS2_MOTOPLUS_EXTERNS_H


#define MP_MEM_PART_SIZE 1048576

extern size_t mpNumBytesFree(void);

extern MP_GRP_ID_TYPE mpCtrlGrpNo2GrpId(int grp_no);

#endif // MOTOROS2_MOTOPLUS_EXTERNS_H
