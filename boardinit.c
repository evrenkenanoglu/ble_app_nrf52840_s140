/** @file       boardinit.c
 *  @brief      Includes Board Init Functions
 *  @author     Evren Kenanoglu
 *  @date       1/27/2021
 */
#define FILE_BOARDINIT_C

/** INCLUDES ******************************************************************/
#include "boardinit.h"


/** CONSTANTS *****************************************************************/

/** TYPEDEFS ******************************************************************/

/** MACROS ********************************************************************/

/** VARIABLES *****************************************************************/

/** LOCAL FUNCTION DECLARATIONS ***********************************************/

/** INTERFACE FUNCTION DEFINITIONS ********************************************/


/**@brief Function for initializing logging. */
 void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
 void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing timers. */
 void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
 void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

void boardInit(void)
{
    log_init();
    leds_init();
    timers_init();
    power_management_init();
}


/** LOCAL FUNCTION DEFINITIONS ************************************************/
