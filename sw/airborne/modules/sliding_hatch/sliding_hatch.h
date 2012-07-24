/*
 * $Id$
 *
 * Copyright (C) 2009  Gautier Hattenberger
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file demo_module.h
 *
 * demo module with blinking LEDs
 */

#ifndef SLIDING_HATCH_H
#define SLIDING_HATCH_H

#include "std.h"
#include "paparazzi.h"

#ifndef SLIDING_HATCH_ON_VALUE
#define SLIDING_HATCH_ON_VALUE 2000
#endif
#ifndef SLIDING_HATCH_OFF_VALUE
#define SLIDING_HATCH_OFF_VALUE 1000
#endif
//#ifndef SLIDING_HATCH_SERVO
//#define SLIDING_HATCH_SERVO SERVO_SWITCH
//#endif

extern int counter_sliding_hatch_open;
extern int counter_sliding_hatch_close;

#define OpenSlidingHatch()  ({ counter_sliding_hatch_open = 10; FALSE; })
#define CloseSlidingHatch() ({ counter_sliding_hatch_close = 10; FALSE; })

void init_sliding_hatch(void);
void periodic_1Hz_sliding_hatch(void);
void start_sliding_hatch(void);
void stop_sliding_hatch(void);

#endif
