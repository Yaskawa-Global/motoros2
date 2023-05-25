//MotoPlusExterns.h

// SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
// SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
//
// SPDX-License-Identifier: Apache-2.0

#ifndef MOTOROS2_MOTOPLUS_EXTERNS_H
#define MOTOROS2_MOTOPLUS_EXTERNS_H


#define MP_MEM_PART_SIZE 1048576

extern int mpNICData(USHORT if_no, ULONG* ip_addr, ULONG* subnet_mask, UCHAR* mac_addr, ULONG* default_gw);
#define	MP_USER_LAN1		1	/* general LAN interface1 */
#define	MP_USER_LAN2		2	/* general LAN interface2(only YRC1000) */

extern size_t mpNumBytesFree(void);

extern MP_GRP_ID_TYPE mpCtrlGrpNo2GrpId(int grp_no);

#endif // MOTOROS2_MOTOPLUS_EXTERNS_H
