/*
 * Code for supporting EMA IPC33xx.
 *
 * Copyright (C) {2012} Guangzhou EMA Tech Co.,Ltd - http://www.ema-tech.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _BOARD_IPC335X_H
#define _BOARD_IPC335X_H

#define IPC335X_CORE            0x0
#define IPC335X_EVM             0x1
#define SOM335X_CORE            0x2

/* REVIST : check posibility of PROFILE_(x) syntax usage */
#define PROFILE_NONE   -1  /* Few board doesn't have profiles */
#define PROFILE_0      (0x1 << 0)
#define PROFILE_1      (0x1 << 1)
#define PROFILE_2      (0x1 << 2)
#define PROFILE_3      (0x1 << 3)
#define PROFILE_4      (0x1 << 4)
#define PROFILE_5      (0x1 << 5)
#define PROFILE_6      (0x1 << 6)
#define PROFILE_7      (0x1 << 7)
#define PROFILE_ALL        0xFF

#endif
