/*
 * Copyright (C) 2024 Korneel Van den Berghe <korneel.vandenberghe@hotmail.be>
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/snn_lander/snn_lander.h"
 * @author Korneel Van den Berghe <korneel.vandenberghe@hotmail.be>
 * Uses a spiking neural network to land the bebop 2 drone. Lander runs at a frequency of 50 Hz. 
 */

#ifndef SNN_LANDER_H
#define SNN_LANDER_H

// for network
// #include "modules/snn_lander/drone_lander.h"
#include <stdbool.h>

struct VerticalInfo {
  float agl;

};

// extern Network snn_net;

extern void init_lander(void);
extern bool land_snn(void);



#endif  // SNN_LANDER_H
