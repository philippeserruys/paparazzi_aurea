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

#include "sliding_hatch.h"
#include "led.h"
#include "servo_switch/servo_switch.h"
#include "generated/airframe.h"
#include "actuators.h"
#include "autopilot.h"

int counter_sliding_hatch_open = 0;
int counter_sliding_hatch_close = 0;

void init_sliding_hatch(void) {
  // this part is already done by led_init in fact
  LED_INIT(POWER_SWITCH_LED);
  //LED_OFF(POWER_SWITCH_LED);
autopilot_SetPowerSwitch(0);
SetServo(SERVO_PWRSWITCH, 1000);
}

void periodic_1Hz_sliding_hatch(void) {
  if (counter_sliding_hatch_open > 0)
  {
     //LED_ON(POWER_SWITCH_LED);
//autopilot_SetPowerSwitch(1);
     SetServo(SLIDING_HATCH_SERVO, SLIDING_HATCH_ON_VALUE);
     SetServo(SERVO_PWRSWITCH, 2000);
     counter_sliding_hatch_open--;
  }
  else if (counter_sliding_hatch_close > 0)
  {
     //LED_ON(POWER_SWITCH_LED);
//autopilot_SetPowerSwitch(1);
     SetServo(SLIDING_HATCH_SERVO, SLIDING_HATCH_OFF_VALUE);
     SetServo(SERVO_PWRSWITCH, 2000);
     counter_sliding_hatch_close--;
  }
  else 
  {
//autopilot_SetPowerSwitch(0);
     SetServo(SERVO_PWRSWITCH, 1000);
     //LED_OFF(POWER_SWITCH_LED);
  }
}


void start_sliding_hatch(void) {
  //LED_ON(DEMO_MODULE_LED);
}

void stop_sliding_hatch(void) {
  //LED_OFF(DEMO_MODULE_LED);
}

