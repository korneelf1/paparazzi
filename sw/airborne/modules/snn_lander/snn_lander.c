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

/** @file "modules/snn_lander/snn_lander.c"
 * @author Korneel Van den Berghe <korneel.vandenberghe@hotmail.be>
 * Uses a spiking neural network to land the bebop 2 drone. Lander runs at a frequency of 50 Hz. 
 */

// for network
// #include "functional.h"
#include "drone_lander.h"
// Header file containing parameters
#include "param/test_lander_conf.h"

#include "snn_lander.h"

// for landing
// #include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "paparazzi.h"
#include "modules/core/abi.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "modules/energy/electrical.h"
#include "navigation.h"
#include "firmwares/rotorcraft/guidance/guidance_hybrid.h"
#include <stdio.h>
#include "state.h"

// #include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
// #include "modules/radio_control/radio_control.h"
// #include "modules/energy/electrical.h"
// #include "firmwares/rotorcraft/guidance/guidance_v.h"
#include <stdbool.h>

#include "paparazzi.h"
#include "modules/sonar/agl_dist.h"
// #include "sw/simulator/nps/nps_sensor_sonar.h"
// #include "modules/sonar/sonar_bebop.h"

/* Default sonar/agl to use */
#ifndef VERTICAL_CTRL_MODULE_AGL_ID
#define VERTICAL_CTRL_MODULE_AGL_ID ABI_BROADCAST
#endif
// PRINT_CONFIG_VAR(VERTICAL_CTRL_MODULE_AGL_ID)

Network snn_net;  // This is the definition of the snn_net variable

struct VerticalInfo v_ctrl;
static abi_event agl_ev; ///< The altitude ABI event

/// Callback function of the ground altitude
static void vertical_ctrl_agl_cb(uint8_t sender_id, uint32_t stamp, float distance);

// Reading from "sensors":
void vertical_ctrl_agl_cb(uint8_t sender_id UNUSED, __attribute__((unused)) uint32_t stamp, float distance)
{
  v_ctrl.agl = distance;

}
// SNN logic

void init_lander(void)
{
  // Build network
  snn_net = build_network(snn_lander_conf.in_size, snn_lander_conf.hid_1_size, snn_lander_conf.hid_2_size, snn_lander_conf.out_size);
  
  // Init network
  init_network(&snn_net);

  // Load network parameters from header file
  load_network_from_header(&snn_net, &snn_lander_conf);
  reset_network(&snn_net);

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(VERTICAL_CTRL_MODULE_AGL_ID, &agl_ev, vertical_ctrl_agl_cb);

  // return net;
}

bool land_snn(void)
{
  // freq = 50.0 Hz
  // retrieve altitude above ground level For some reason always off by 0.6 m
  float agl = agl_dist_value;

  printf("Altitude filtered: %.2f\n", agl);
  // printf("Altitude not filtered: %.2f\n", agl_dist_value);
  // printf("Altitude from raw sonar: %.2f\n", sonar_bebop.distance);
  // printf("From state: %.2f\n",state.alt_agl_f);
  if (agl < 0.1 ){
    printf("Landed\n");
    guidance_v_set_th(0.);
    return false;
  }

  else {
    // Set input to network
    for (int i = 0; i < snn_lander_conf.in_size; i++) {
      snn_net.in[i] = agl;
    }

    float thrust_set = forward_network(&snn_net);
    float throttle_set;
    printf("thrust_set by snn: %.2f\n", thrust_set);
    // Set output to stabilization
    // printf("current command thrust: %.i\n", stabilization_cmd[COMMAND_THRUST]);
    // stabilization_cmd[COMMAND_THRUST] = (int32_t) thrust_set;
    // stabilization_cmd[COMMAND_THRUST] = (int32_t) 9000;
    // printf("After setting command thrust: %.i\n", stabilization_cmd[COMMAND_THRUST]);
    // other attempts involved nav.throttle or setting gv_zdd from guidance_v
    throttle_set = 0.5+(thrust_set-3.5)/7;
    
    printf("Deviation from hover: %.2f\n", (thrust_set-3.5)/7);
    printf("Throttle setting: %.4f\n", throttle_set);
    if(throttle_set > 0.7){
      throttle_set = 0.7;
    }
    guidance_v_set_th(throttle_set); // throttle setting between 0 and 1
    
    return true;
  }
}


