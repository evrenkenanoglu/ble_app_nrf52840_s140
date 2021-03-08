/** @file       bleall.c
 *  @brief      All ble functions, definitions, parameters... 
 *  @author     Evren Kenanoglu
 *  @date       1/27/2021
 */
#define FILE_BLEALL_C

/** INCLUDES ******************************************************************/
#include "bleall.h"

/** CONSTANTS *****************************************************************/

/** TYPEDEFS ******************************************************************/

/** MACROS ********************************************************************/

/** VARIABLES *****************************************************************/

/** LOCAL FUNCTION DECLARATIONS ***********************************************/

/** INTERFACE FUNCTION DEFINITIONS ********************************************/

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init(tsBleParams *params)
{
    uint32_t err_code;
    uint8_t flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *)m_beacon_info;
    manuf_specific_data.data.size   = sizeof(m_beacon_info);//APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&params->advdata, 0, sizeof(params->advdata));

    params->advdata.name_type             = BLE_ADVDATA_NO_NAME;
    params->advdata.flags                 = flags;
    params->advdata.p_manuf_specific_data = &manuf_specific_data;
    *(params->advdata.p_tx_power_level) = params->txPower;

    // Initialize advertising parameters (used when starting advertising).
    memset(&params->m_adv_params, 0, sizeof(params->m_adv_params));

    params->m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED; //BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    params->m_adv_params.p_peer_addr     = NULL;                                                 // Undirected advertisement.
    params->m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    params->m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    params->m_adv_params.duration        = 0; // Never time out.

    err_code = ble_advdata_encode(&params->advdata, params->m_adv_data.adv_data.p_data, &params->m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&params->m_adv_handle, &params->m_adv_data, &params->m_adv_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
ret_code_t advertising_start(tsBleParams *params)
{
    ret_code_t errCode;

    errCode = sd_ble_gap_adv_start(params->m_adv_handle, APP_BLE_CONN_CFG_TAG);
    VERIFY_SUCCESS(errCode);

    errCode = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    VERIFY_SUCCESS(errCode);

    params->bleAdvStatus = eBleAdvertising;
    
    return errCode;
}

/**
 * @brief Function for stop advertising.
 * 
 * @param params BLE structure object pointer
 */
ret_code_t advertising_stop(tsBleParams *params)
{
    ret_code_t errCode;
    errCode = sd_ble_gap_adv_stop(params->m_adv_handle);
    VERIFY_SUCCESS(errCode);

    errCode = bsp_indication_set(BSP_INDICATE_IDLE);
    VERIFY_SUCCESS(errCode);

    params->bleAdvStatus = eBleIdle;

    return errCode;
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(tsBleParams *params)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code           = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);



}

void ble_params_init(tsBleParams *params)
{
    params->m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */

    /**@brief Struct that contains pointers to the encoded advertising data. */
    params->m_adv_data.adv_data.p_data = params->m_enc_advdata;
    params->m_adv_data.adv_data.len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

    params->m_adv_data.scan_rsp_data.p_data = NULL;
    params->m_adv_data.scan_rsp_data.len    = 0;

    params->bleAdvStatus = eBleIdle;
    params->txPower      = POWER_TX_LEVEL_0_DB;



}

ret_code_t bleAdvUpdateData(tsBleParams *params, void *updateData, uint32_t updateDataSize)
{
    ret_code_t errCode;

    if(params->bleAdvStatus != eBleIdle)
    {
      errCode = advertising_stop(params);
      VERIFY_SUCCESS(errCode);
    }
    uint8_t flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_t newAdvData;
    ble_advdata_manuf_data_t manuf_specific_data;

    memset(&newAdvData, 0, sizeof(newAdvData));

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data        = (uint8_t *)updateData;
    manuf_specific_data.data.size          = updateDataSize;

    newAdvData.name_type             = BLE_ADVDATA_FULL_NAME;
    newAdvData.flags                 = flags;
    newAdvData.p_manuf_specific_data = &manuf_specific_data;
    *(newAdvData.p_tx_power_level) = params->txPower;


    sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, params->m_adv_handle, params->txPower);

    errCode = ble_advdata_encode(&newAdvData, params->m_adv_data.adv_data.p_data, &params->m_adv_data.adv_data.len);
    VERIFY_SUCCESS(errCode);

    errCode = sd_ble_gap_adv_set_configure(&params->m_adv_handle, &params->m_adv_data, &params->m_adv_params);
    VERIFY_SUCCESS(errCode);
  
    if(params->bleAdvStatus != eBleAdvertising)
    {
      errCode = advertising_start(params);
      VERIFY_SUCCESS(errCode);
    }
    return errCode;
}

/**
 * @brief 
 * 
 * @param p_scan_evt 
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;
    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_WHITELIST_REQUEST:
        {
            
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {

        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            printf("Scan timed out.");
        } break;

        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
            break;
        case NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT:
            break;

        default:
          break;
    }
}


ret_code_t bleScanInit(tsBleScanParams *params)
{
    ret_code_t errCode;
    memset(&params->initScan, 0, sizeof(params->initScan));

    params->initScan.p_scan_param     = &params->scanParam;
    params->initScan.connect_if_match = false;
    params->initScan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    errCode = nrf_ble_scan_init(params->scanModule, &params->initScan, params->scanEventHandler);
    VERIFY_SUCCESS(errCode);

    return errCode;
}

ret_code_t bleScanStart(tsBleScanParams *params)
{
    ret_code_t errCode;
    errCode = nrf_ble_scan_params_set(params->scanModule, &params->scanParam);
    VERIFY_SUCCESS(errCode);

    errCode = nrf_ble_scan_start(params->scanModule);
    VERIFY_SUCCESS(errCode);

    return errCode;
}

void bleScanStop(tsBleScanParams *params)
{
    nrf_ble_scan_stop(); 
}


ret_code_t bleScanFilterSet(tsBleScanParams *params, tsBleScanFilters *paramsFilter)
{
    ret_code_t errCode;
    errCode = nrf_ble_scan_filter_set(params->scanModule, paramsFilter->filterType, paramsFilter->filter);
    return errCode;
}

ret_code_t bleScanFiltersEnable(tsBleScanParams *params)
{
    ret_code_t errCode;
    errCode = nrf_ble_scan_filters_enable(params->scanModule, NRF_BLE_SCAN_NAME_FILTER, false);
    return errCode;
}

ret_code_t bleScanFiltersDisable(tsBleScanParams *params)
{
    ret_code_t errCode;
    errCode = nrf_ble_scan_filters_disable(params->scanModule);
    return errCode;
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(const char *deviceName)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;

    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)deviceName,
                                          strlen(deviceName));
    APP_ERROR_CHECK(err_code);

    ///* YOUR_JOB: Use an appearance value matching the application's use case.
    //   err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    //   APP_ERROR_CHECK(err_code); */

    //memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    //gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    //gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    //gap_conn_params.slave_latency     = SLAVE_LATENCY;
    //gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    //err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    //APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
void gatt_init(tsBleParams *params)
{
    ret_code_t err_code = nrf_ble_gatt_init(params->gatt, NULL);
    APP_ERROR_CHECK(err_code);
}
// ret_code_t bleAdvertisingAdvdataUpdate(tsBleParams *params, void *updateData)
// {
//     uint8_t flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

//     VERIFY_PARAM_NOT_NULL(params);
//     // if (p_advertising->initialized == false)
//     // {
//     //     return NRF_ERROR_INVALID_STATE;
//     // }

//     if ((p_advdata == NULL) && (p_srdata == NULL))
//     {
//         return NRF_ERROR_NULL;
//     }

//     ble_gap_adv_data_t newAdvData;
//     memset(&new_adv_data, 0, sizeof(new_adv_data));

//     if (updateData != NULL)
//     {
//         new_adv_data.adv_data.p_data =
//             (p_advertising->p_adv_data->adv_data.p_data != p_advertising->enc_advdata[0]) ?
//              p_advertising->enc_advdata[0] : p_advertising->enc_advdata[1];
//         new_adv_data.adv_data.len = adv_set_data_size_max_get(p_advertising);

//         newAdvdata.name_type             = BLE_ADVDATA_NO_NAME;
//         newAdvdata.flags                 = flags;
//         newAdvdata.p_manuf_specific_data = &manuf_specific_data;

//         ret_code_t ret = ble_advdata_encode(p_advdata,
//                                             new_adv_data.adv_data.p_data,
//                                             &new_adv_data.adv_data.len);
//         VERIFY_SUCCESS(ret);
//     }

//     // if (p_srdata != NULL)
//     // {
//     //     new_adv_data.scan_rsp_data.p_data =
//     //         (p_advertising->p_adv_data->scan_rsp_data.p_data != p_advertising->enc_scan_rsp_data[0]) ?
//     //          p_advertising->enc_scan_rsp_data[0] : p_advertising->enc_scan_rsp_data[1];
//     //     new_adv_data.scan_rsp_data.len = adv_set_data_size_max_get(p_advertising);

//     //     ret_code_t ret = ble_advdata_encode(p_srdata,
//     //                                         new_adv_data.scan_rsp_data.p_data,
//     //                                         &new_adv_data.scan_rsp_data.len);
//     //     VERIFY_SUCCESS(ret);
//     // }

//     memcpy(&params->advdata, &new_adv_data, sizeof(params->advdata));

//     params->ad->p_adv_data = &p_advertising->adv_data;

//     return sd_ble_gap_adv_set_configure(&params->m_adv_handle,
//                                         p_advertising->p_adv_data,
//                                         NULL);
// }

//ret_code_t BLE_AdvertisingStop(void)
//{
//    ret_code_t err_code = NRF_SUCCESS;

//    if (m_adv_handle != BLE_GAP_ADV_SET_HANDLE_NOT_SET)
//    {
//        printf("BLE Stop advertising.\n");
//        err_code = sd_ble_gap_adv_stop(m_adv_handle);
//        if (err_code != NRF_ERROR_INVALID_STATE)
//        {
//            APP_ERROR_CHECK(err_code);
//        }
//        m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
//    }

//    return err_code;
//}



/** LOCAL FUNCTION DEFINITIONS ************************************************/
