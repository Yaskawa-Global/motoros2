//MotoPlusExterns.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_MOTOPLUS_EXTERNS_H
#define MOTOROS2_MOTOPLUS_EXTERNS_H


#define MP_MEM_PART_SIZE 1048576

extern size_t mpNumBytesFree(void);

#if defined (YRC1000) || defined (YRC1000u)
extern MP_GRP_ID_TYPE mpCtrlGrpNo2GrpId(int grp_no);
#endif

//M+ only defines MAX_ALARM_COUNT, but we'd like to avoid magic nrs
//as much as possible, so we define our own here for errors.
//But only if something hasn't been defined already.
#ifndef MAX_ERROR_COUNT
#define MAX_ERROR_COUNT 1
#endif

#ifndef E_EXRCS_PFL_FUNC_BUSY
#define E_EXRCS_PFL_FUNC_BUSY               (-19)
#endif

#ifndef E_EXRCS_UNDER_ENERGY_SAVING
#define E_EXRCS_UNDER_ENERGY_SAVING         (-20)
#endif

#endif // MOTOROS2_MOTOPLUS_EXTERNS_H
