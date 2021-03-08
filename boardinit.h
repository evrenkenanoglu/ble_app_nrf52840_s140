/** @file       boardinit.h
 *  @brief      Includes Board Init Functions
 *  @author     Evren Kenanoglu
 *  @date       1/27/2021
 */
#ifndef FILE_BOARDINIT_H
#define FILE_BOARDINIT_H

/** INCLUDES ******************************************************************/

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
/** CONSTANTS *****************************************************************/

/** TYPEDEFS ******************************************************************/

/** MACROS ********************************************************************/

#ifndef FILE_BOARDINIT_C
#define INTERFACE extern
#else
#define INTERFACE
#endif

/** VARIABLES *****************************************************************/

/** FUNCTIONS *****************************************************************/

INTERFACE void log_init();
INTERFACE void timers_init();
INTERFACE void leds_init();
INTERFACE void power_management_init();
INTERFACE void boardInit();

#undef INTERFACE // Should not let this roam free

#endif // FILE_BOARDINIT_H
