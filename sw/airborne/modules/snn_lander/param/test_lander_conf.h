#pragma once

#include "modules/snn_lander/drone_lander.h" // for NetworkConf

// Include child structs
#include "modules/snn_lander/param/test_connection_conf_hidout.h"
#include "modules/snn_lander/param/test_connection_conf_inhid.h"
#include "modules/snn_lander/param/test_connection_conf_hidhid.h"
#include "modules/snn_lander/param/test_lif_conf_1.h"
#include "modules/snn_lander/param/test_lif_conf_2.h"
#include "modules/snn_lander/param/test_lif_conf_3.h"

extern NetworkConf const snn_lander_conf = {1,7,32,32,
  &conf_inhid, &conf_inhidlif, &conf_hidhid, &conf_hidhidlif, &conf_hidout, &conf_hidoutlif};