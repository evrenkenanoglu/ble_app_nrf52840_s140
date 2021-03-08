/** @file       bleall.h
 *  @brief      All ble functions, definitions, parameters... 
 *  @author     Evren Kenanoglu
 *  @date       1/27/2021
 */
#ifndef FILE_BLEALL_H
#define FILE_BLEALL_H

/** INCLUDES ******************************************************************/
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "boardinit.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "ble_gap.h"


/** CONSTANTS *****************************************************************/
#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */
#define DEVICE_NAME                     "Nordic_Evren_1"                       /**< Name of device. Will be included in the advertising data. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x18                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                  0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//** TX POWER LEVEL **//
#define POWER_TX_LEVEL_MINUS_40_DB (-40)
#define POWER_TX_LEVEL_0_DB        (0)
#define POWER_TX_LEVEL_4_DB        (4)

//** GAP DEFINES **//
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

//** SCAN DEFINES **//
#define APP_BLE_CONN_CFG_TAG      1                                /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO     3                                /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO     1                                /**< Applications' SoC observer priority. You shouldn't need to modify this value. */


#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */



#endif

/** TYPEDEFS ******************************************************************/

/** MACROS ********************************************************************/

#ifndef FILE_BLEALL_C
#define INTERFACE extern
#else
#define INTERFACE
#endif

typedef enum 
{
    eBleIdle =0,
    eBleAdvertising,
    eBleScanning,
}teBleAdvertisingCurrentStatus;

typedef enum
{
    eTxPowerMinus40Dbm,
    eTxPowerMinus20Dbm,
    eTxPowerMinus16Dbm,
    eTxPowerMinus12Dbm,
    eTxPowerMinus8Dbm,
    eTxPowerMinus4Dbm,
    eTxPower0Dbm,
    eTxPower3Dbm,
    eTxPower4Dbm,
}teBleAdvTxPower;

typedef struct
{
    nrf_ble_scan_init_t initScan;
    ble_gap_scan_params_t scanParam;
    nrf_ble_scan_t const *scanModule;
    void (*scanEventHandler)(void);
} tsBleScanParams;

typedef struct
{
    uint8_t filterType;
    const void *filter;
}tsBleScanFilters;

typedef struct 
{
    uint8_t bleAdvStatus;
    uint8_t m_adv_handle;
    uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    ble_gap_adv_params_t m_adv_params;
    ble_gap_adv_data_t m_adv_data;
    ble_advdata_t advdata;
    int8_t txPower;
    nrf_ble_gatt_t *gatt;
    void (*bleEventHandler)(void);
}tsBleParams;



typedef struct 
{
    uint32_t dummy_Value;
}tsAdvertisementDataPackage;


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] = /**< Information advertised by the Beacon. */
    {
        APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                             // implementation.
        APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                             // manufacturer specific data in this implementation.
        APP_BEACON_UUID,     // 128 bit UUID value.
        APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
        APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
        APP_MEASURED_RSSI,    // Manufacturer specific information. The Beacon's measured TX power in
                             // this implementation.
        0x26,
    
};

/** FUNCTIONS *****************************************************************/
INTERFACE void ble_stack_init(tsBleParams *params);
INTERFACE void ble_params_init(tsBleParams *params);
INTERFACE void advertising_init(tsBleParams *params);
INTERFACE void gatt_init(tsBleParams *params);
INTERFACE void gap_params_init(const char *deviceName);

INTERFACE ret_code_t advertising_start(tsBleParams *params);
INTERFACE ret_code_t advertising_stop(tsBleParams *params);
INTERFACE ret_code_t bleAdvUpdateData(tsBleParams *params, void *updateData, uint32_t sizeofData);

INTERFACE ret_code_t bleScanInit(tsBleScanParams *params);
INTERFACE ret_code_t bleScanStart(tsBleScanParams *params);
INTERFACE void bleScanStop(tsBleScanParams *params);
INTERFACE ret_code_t bleScanFilterSet(tsBleScanParams *params, tsBleScanFilters *paramsFilter);
INTERFACE ret_code_t bleScanFiltersEnable(tsBleScanParams *params);
INTERFACE ret_code_t bleScanFiltersDisable(tsBleScanParams *params);

#undef INTERFACE // Should not let this roam free

#endif // FILE_BLEALL_H
