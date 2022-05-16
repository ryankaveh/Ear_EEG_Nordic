/**
// peripheral_v6 functionality:

IMPLEMENTED & TESTED:
- Bluetooth (with data throuput of up to 1.2 Mbps)
--> BLE optimizations include packing 3 SPI packets into a single BLE packet
--> Stack optimizations that queue a number of SPI packets
--> Double buffer scheme so that BLE and SPI can operate independantly and not block eachother

- SPI transactions with easydma (NRFX_SPIM)
- Timers to set interval between spi transactions (aka rate of spi transactions)
- Incoming Instruction decoding:
--> Based off BLE commands this FW can:
-------> [led #]   toggle LEDs
-------> [read #]  Parse incoming R commands and construct SPI trx array with appropriate data
-------> [write #] Parse incoming W commands and construct SPI trx array with appropriate data
-------> [start]   Can start a timer to stream data to host device
-------> [stop]    Can stop a timer that is streaming data to host device
-------> [update]  Toggle BLE into full speed (2M PHY)
-------> [startup] start a timer to run through startup instructions that are stored in memory

- Currently working with NRF52 dev kits as peripheral device and NRF52 Dongle as base station


TO TEST:
- Using bringup board's own NRF52840 as the peripheral device
 */


#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nordic_common.h"


#include "ble_nus.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"

#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_timer.h"
#include "app_fifo.h"
#include "app_uart.h"
#include "app_util_platform.h"

#include "bsp_btn_ble.h"

////////////////////////// added for SPI //////////////////////////
#include "nrfx_spim.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"
#include "app_error.h"
#include "app_util.h"
//////////////////////////              //////////////////////////

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(50, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define LED_BLE_NUS_RX   (BSP_BOARD_LED_1)
#define ENDLINE_STRING "\r\n"
#define BLE_LEN                 560
#define MAX_BUFF_LEN            300

APP_TIMER_DEF(trx_timer_id);      // SPI transaction timer
APP_TIMER_DEF(wakeup_timer_id);  // start up timer

/*
#define NRFX_SPIM_SCK_PIN  3
#define NRFX_SPIM_MOSI_PIN 4
#define NRFX_SPIM_MISO_PIN 28
#define NRFX_SPIM_SS_PIN   31
#define NRFX_SPIM_DCX_PIN  30
*/

// dev kit has the following pinnout:
#define NRFX_SPIM_SS_PIN   29
#define NRFX_SPIM_SCK_PIN  26
#define NRFX_SPIM_MOSI_PIN 2
#define NRFX_SPIM_MISO_PIN 31

// dev kit has 3 options for MOSI for some reason
//  p0.02 -> 2
//  p0.28 -> 28
//  p0.30 -> 38

// dev kit LEDs are:
//  LED1 p0.13
//  LED2 p0.22
//  LED3 p0.15
//  LED4 p0.24

#define SPI_IDLE            0
#define SPI_STREAM          1
#define SPI_READ_REG        2
#define SPI_WRITE_REG       3
#define SPI_READ_SINGLE     4
#define SPI_WAKEUP_CHIP     5


#define SPI_INSTANCE  3                                           /**< SPI instance index. */
static const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};
static char           m_nus_data_array[BLE_NUS_MAX_DATA_LEN];
static bool           command_rdy;
static const char     delimiter[2] = " ";
static char           m_tx_buffer[BLE_LEN];
static char           error_buf[BLE_LEN];
static uint8_t        ble_tx_buff_t[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint8_t        test_counter = 0;
static uint8_t        toggle = 0;
static uint32_t       timer_ticks = 10; // 8 ticks from a 32kHz clock should give me a transaction every ~500us
static uint32_t       wakeup_timer_ticks = 1000; // time between start up instructions
static uint8_t        timer_state = 0;
static bool           trx_rdy_for_ble;
static uint32_t       ble_ready = NRF_SUCCESS;

// flag for understanding what spi_cmd we have to satisfy
static uint8_t        spi_cmd_flag = 0;
static uint8_t        spi_trx_counter = 0;
static uint8_t        addr         = 0;
static char           wr_data_full[] = "BEEF";
static char           wr_data_a[] = "DE";
static char           wr_data_b[] = "AD";
static char           rx_buff_hex[131];
static uint16_t       rx_buff_hex_size = sizeof(rx_buff_hex);

static uint16_t       counter_size = sizeof(uint8_t);

static uint8_t       instr_counter = 0;

// because of write command bit, element 0 is 64 + addr e.g. 73 = addr 9
static uint8_t       wakeup_instr[29][4] = {{73,0,8,0}, 
                                            {74,0,0,16},
                                            {75,0,3,232},
                                            {76,0,0,11},
                                            {84,0,130,253},
                                            {85,0,1,126},
                                            {86,0,136,136},
                                            {87,0,136,136},
                                            {88,0,136,136},
                                            {89,0,136,136},
                                            {90,0,204,204},
                                            {91,0,204,204},
                                            {92,0,68,68},
                                            {92,0,68,68},
                                            {94,0,0,0},
                                            {95,0,0,0},
                                            {96,0,51,51},
                                            {97,0,51,51},
                                            {98,0,170,170},
                                            {99,0,170,170},
                                            {100,0,0,255},
                                            {114,0,48,97},
                                            {119,0,8,134},
                                            {120,0,85,85},
                                            {121,0,153,153},
                                            {124,0,255,255},
                                            {125,0,255,255},
                                            {126,0,0,0},
                                            {64,0,255,59}
                                            };

// SPI transfer buffer
static uint8_t       spi_tx_buf[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0};
static uint8_t       spi_rx_buf[sizeof(spi_tx_buf) + 1];        /**< RX buffer. */

static uint16_t      spi_length = sizeof(spi_tx_buf);        /**< Transfer length. */
//static uint16_t      spi_length = sizeof(uint8_t)*65;

// Will group 3 SPI packets into a single BLE packet
static uint8_t       ble_tx_buf[] = {128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,
                                   128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,
                                   128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0};

static uint8_t       ble_tx_buf_send[] = {128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,
                                    128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0,
                                    128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                                    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,16,0,0,0};

static uint16_t      ble_packet_length = sizeof(ble_tx_buf);      // BLE Transfer length

static uint8_t       read_data_instr[4] = {128,0,0,0};            // SPI Read command values
//static uint8_t       read_data_instr[4] = {0,0,0,0};            // SPI Read command values

static uint8_t       prev_packet_id;
static uint8_t       current_packet_id;

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

 void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

void spim_event_handler(nrfx_spim_evt_t const * p_event,
                       void *                  p_context)
{
    spi_xfer_done = true;
    // Bring CS high
    nrf_gpio_pin_set(NRFX_SPIM_SS_PIN);
    //NRF_LOG_INFO("SPI TRX COMPLETE");
    
    if (spi_cmd_flag == SPI_READ_REG) {
      NRF_LOG_INFO("READ REG CCOMPLETE");
      spi_cmd_flag = 0;

      while(ble_nus_data_send(&m_nus, spi_rx_buf, &spi_length, m_conn_handle) != NRF_SUCCESS);
    } else if (spi_cmd_flag == SPI_WRITE_REG) {
      NRF_LOG_INFO("WRITE REG CCOMPLETE");
      spi_cmd_flag = 0;
      //uint16_t size2 = sprintf(m_tx_buffer, "WRITE REG COMPLETE\r\n");
      //while(ble_nus_data_send(&m_nus, (uint8_t *) m_tx_buffer, &size2, m_conn_handle) != NRF_SUCCESS);

    } else if (spi_cmd_flag == SPI_READ_SINGLE) {
      NRF_LOG_INFO("READ SINGLE DATA CCOMPLETE");
      spi_cmd_flag = 0;

      memcpy((uint8_t *) ble_tx_buff_t, spi_rx_buf, spi_length); //non_char
      //NRF_LOG_INFO("packet_id: %d", ble_tx_buff_t[0]);
      while(ble_nus_data_send(&m_nus, ble_tx_buff_t, &spi_length, m_conn_handle) != NRF_SUCCESS); //non_char
    } else if (spi_cmd_flag == SPI_WAKEUP_CHIP) {
      NRF_LOG_INFO("WAKEUP INSTR SENT");
    } else if (spi_cmd_flag == SPI_STREAM) {
     
      spi_trx_counter = spi_trx_counter+1;
   
      if (spi_trx_counter == 3)
      {
        // we've filled up the buffer, transfer the data to the send buffer so we can fill up the spi buffer again
        memcpy((uint8_t *) ble_tx_buf_send, ble_tx_buf, ble_packet_length); //non_char
        spi_trx_counter = 0;

        // if the bluetooth is ready right now, try sending it. If it isn't ready, we'll send it later 
        if (ble_ready == NRF_SUCCESS)
        {
          //NRF_LOG_INFO("Sending data");
          ble_ready = ble_nus_data_send(&m_nus, ble_tx_buf_send, &ble_packet_length, m_conn_handle);
        }
      }
    }
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


static void trx_timer_handler(void * p_context)
{
  // update spi_tx_buff to have read data commands
  spi_tx_buf[0]   = read_data_instr[0];
  spi_tx_buf[1]   = read_data_instr[1];
  spi_tx_buf[63]  = read_data_instr[2];
  spi_tx_buf[64]  = read_data_instr[3];

  // fire off a SPI transaction
  //NRF_LOG_INFO("%s", spi_tx_buf);

  // bring CS low in anticipation
  nrf_gpio_pin_clear(NRFX_SPIM_SS_PIN);
  //nrf_delay_us(5);

  if (spi_xfer_done == true)
  {
    spi_xfer_done = false;
    if (spi_trx_counter == 0)
    {
      nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(spi_tx_buf, spi_length, ble_tx_buf, spi_length);
      APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));
   
    } else if (spi_trx_counter == 1) {
      nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(spi_tx_buf, spi_length, ble_tx_buf+65, spi_length);
      APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));

    } else if (spi_trx_counter == 2) {
      nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(spi_tx_buf, spi_length, ble_tx_buf+130, spi_length);
      APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));

    }
  }
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * BLE EVENT HANDLER
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    // If we're here we've recieved data over BLE
    // The data should be residing in params.rx_data.p_data and have length of p_evt->params.rx_data.length
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        
        bsp_board_led_invert(LED_BLE_NUS_RX);                                                     // Toggle LED to show we've recieved data
        memcpy(m_nus_data_array, p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);     // Make a copy of this data in a buffer array

        if (p_evt->params.rx_data.length + sizeof(ENDLINE_STRING) < BLE_NUS_MAX_DATA_LEN)
        {
            memcpy(m_nus_data_array + p_evt->params.rx_data.length, ENDLINE_STRING, sizeof(ENDLINE_STRING));
            //length += sizeof(ENDLINE_STRING);
        }
        command_rdy = 1;
    }

}

static void wakeup_chip()
{
  spi_tx_buf[0]   = wakeup_instr[instr_counter][0];
  spi_tx_buf[1]   = wakeup_instr[instr_counter][1];
  spi_tx_buf[63]  = wakeup_instr[instr_counter][2];
  spi_tx_buf[64]  = wakeup_instr[instr_counter][3];
  instr_counter = instr_counter + 1;
  
  nrf_gpio_pin_clear(NRFX_SPIM_SS_PIN);
  nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(spi_tx_buf, spi_length, spi_rx_buf, spi_length);
  APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));
  //nrfx_spim_xfer(&spi, spi_tx_buf, spi_length, spi_rx_buf, spi_length);
}

static void wakeup_timer_handler(void * p_context)
{
    if (instr_counter < 29)
    {
      NRF_LOG_INFO("instr: %d", instr_counter);
      bsp_board_led_invert(BSP_BOARD_LED_3);
      wakeup_chip();
    }
    else
    {
      instr_counter = 0;
      app_timer_stop(wakeup_timer_id);
      spi_cmd_flag = SPI_IDLE;
    }
}

static void start_trx_timer()
{
  timer_state = 1;
  app_timer_start(trx_timer_id, timer_ticks, NULL);
}

static void stop_trx_timer()
{
  timer_state = 0;
  app_timer_stop(trx_timer_id);
}

static void process_rx_buffer()
{
  //strtok through the rx_buff
  char *token;
  token = strtok(m_nus_data_array, delimiter);
  NRF_LOG_INFO("token 1: %s", token);
  if (strcmp(token,"led") == 0) {
    NRF_LOG_INFO("LED cmd received");
    token = strtok(NULL, delimiter);
    switch(atoi(token))
    {
      case 0:   bsp_board_led_invert(BSP_BOARD_LED_0); break;
      case 1:   bsp_board_led_invert(BSP_BOARD_LED_1); break;
      case 2:   bsp_board_led_invert(BSP_BOARD_LED_2); break;
      default:  bsp_board_led_invert(BSP_BOARD_LED_3); break;
    }
  }
  else if (strcmp(token,"start") == 0) {
    NRF_LOG_INFO("start cmd received");
    spi_cmd_flag = SPI_STREAM;
    start_trx_timer();
  }
  else if (strcmp(token,"stop") == 0) {
    NRF_LOG_INFO("start cmd received");
    spi_cmd_flag = SPI_READ_REG;
    stop_trx_timer();
  }
  else if (strcmp(token,"startup") == 0)
  {
    spi_cmd_flag = SPI_WAKEUP_CHIP;
    app_timer_start(wakeup_timer_id, wakeup_timer_ticks, NULL);
  }
  else if (strcmp(token,"read") == 0) {
    token = strtok(NULL, delimiter);
    addr = (uint8_t) atoi(token);

    spi_tx_buf[0]   = addr;
    spi_tx_buf[1]   = 0;
    spi_tx_buf[63]  = 0;
    spi_tx_buf[64]  = 0;
    
    spi_cmd_flag = SPI_READ_REG; //spi_cmd_flag = read

    // shoot off SPI_TRX
    if (spi_xfer_done == true)
    {
      nrf_gpio_pin_clear(NRFX_SPIM_SS_PIN);
      spi_xfer_done = false;
      nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(spi_tx_buf, spi_length, spi_rx_buf, spi_length);
      APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));
    }
    
  }
  else if (strcmp(token,"write") == 0) {

    token = strtok(NULL, delimiter);
    addr = atoi(token);
    
    token = strtok(NULL, delimiter);
    // what happens if token is more than 4 chars?
    //wr_data_full = token
    memcpy(wr_data_full, token, 4*sizeof(*wr_data_full)); 
    wr_data_a[0] = wr_data_full[0];
    wr_data_a[1] = wr_data_full[1];
    wr_data_b[0] = wr_data_full[2];
    wr_data_b[1] = wr_data_full[3];

    spi_tx_buf[0]   = addr + 64; //add 64 to turn 'write' bit high
    spi_tx_buf[1]   = 0;
    spi_tx_buf[63]  = (uint8_t)strtol(wr_data_a, NULL, 16);
    spi_tx_buf[64]  = (uint8_t)strtol(wr_data_b, NULL, 16);

    spi_cmd_flag = SPI_WRITE_REG; //spi_cmd_flag = write

    // shoot off SPI_TRX
    if (spi_xfer_done == true)
    {
      nrf_gpio_pin_clear(NRFX_SPIM_SS_PIN);
      spi_xfer_done = false;
      nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(spi_tx_buf, spi_length, spi_rx_buf, spi_length);
      APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));
    }
  }
  else if (strcmp(token,"single") == 0) {
    NRF_LOG_INFO("single data read cmd recieved");
    
    token = strtok(NULL, delimiter);
    NRF_LOG_INFO("read token: %s", token);
    addr = (uint8_t) atoi(token);
    NRF_LOG_INFO("read addr: %d", addr);

    spi_tx_buf[0]   = read_data_instr[0];
    spi_tx_buf[1]   = read_data_instr[1];
    spi_tx_buf[63]  = read_data_instr[2];
    spi_tx_buf[64]  = read_data_instr[3];
    
    spi_cmd_flag = SPI_READ_SINGLE; //spi_cmd_flag = read single trx
    NRF_LOG_INFO("cmd: %d - %d ... %d - %d",spi_tx_buf[0], spi_tx_buf[1], spi_tx_buf[63], spi_tx_buf[64]);
    if (spi_xfer_done == true)
    {
      nrf_gpio_pin_clear(NRFX_SPIM_SS_PIN);
      NRF_LOG_INFO("SENDING SPI CMD...");
      spi_xfer_done = false;
      nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(spi_tx_buf, spi_length, spi_rx_buf, spi_length);
      APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));
    }
  }
  else if (strcmp(token,"reset") == 0) {
    sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  }
  else if (strcmp(token,"update") == 0) {
  uint32_t err_code;
  NRF_LOG_INFO("PHY update request.");
  ble_gap_phys_t const phys =
  {
      .rx_phys = BLE_GAP_PHY_2MBPS,
      .tx_phys = BLE_GAP_PHY_2MBPS,
  };
  err_code = sd_ble_gap_phy_update(m_conn_handle, &phys);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, 251);
  APP_ERROR_CHECK(err_code);

  } else {
    uint16_t size2 = sprintf(error_buf, "Invalid instruction. Make sure you end with a space. Valid instructions are:\r\nLED \r\nThat's it for now...\r\n");
    while(ble_nus_data_send(&m_nus, (uint8_t *) error_buf, &size2, m_conn_handle) != NRF_SUCCESS);
  }
}

/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    bool conn_evt_len_ext_enabled;
    ble_opt_t  opt;

    conn_evt_len_ext_enabled = true;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = conn_evt_len_ext_enabled ? 1 : 0;

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);

    uint8_t tx_power = 8;
    //tx_power_set(tx_power);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BSP_EVENT_KEY_0: ;
            uint16_t size4 = sprintf(m_tx_buffer, "BSP event key 0\r\n");
            while(ble_nus_data_send(&m_nus, (uint8_t *) m_tx_buffer, &size4, m_conn_handle) != NRF_SUCCESS);
            break;

        case BSP_EVENT_KEY_1: ;
            uint16_t size3 = sprintf(m_tx_buffer, "BSP event key 1\r\n");
            while(ble_nus_data_send(&m_nus, (uint8_t *) m_tx_buffer, &size3, m_conn_handle) != NRF_SUCCESS);
            break;

        default:
            break;
    }
}


void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

void tx_power_set(int8_t tx_power)
{
	uint32_t err_code;
	err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, tx_power);
	APP_ERROR_CHECK(err_code);
}

static void spi_init(void)
{
    ret_code_t err_code;



    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;//NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_8M;
    // spi_config.ss_pin         = NRFX_SPIM_SS_PIN;
    spi_config.ss_pin         = NRFX_SPIM_PIN_NOT_USED;
    spi_config.miso_pin       = NRFX_SPIM_MISO_PIN;
    spi_config.mosi_pin       = NRFX_SPIM_MOSI_PIN;
    spi_config.sck_pin        = NRFX_SPIM_SCK_PIN;
    //spi_config.dcx_pin        = NRFX_SPIM_DCX_PIN;
    spi_config.dcx_pin        = NRFX_SPIM_PIN_NOT_USED;
    spi_config.use_hw_ss      = true;
    spi_config.ss_active_high = false;
    APP_ERROR_CHECK(nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL));


    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    uart_init();
    
    //nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(spi_tx_buf, spi_length, spi_rx_buf, spi_length);
    
    nrf_gpio_cfg_output(NRFX_SPIM_SS_PIN);
    nrf_gpio_pin_set(NRFX_SPIM_SS_PIN);

    spi_init();
    spi_xfer_done = true;

    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    NRF_LOG_INFO("Debug logging for UART over RTT started.");

    NRF_LOG_INFO("Initializching Timers");
    app_timer_create(&trx_timer_id, APP_TIMER_MODE_REPEATED, trx_timer_handler);
    app_timer_create(&wakeup_timer_id, APP_TIMER_MODE_REPEATED, wakeup_timer_handler);

    NRF_LOG_INFO("Advertising...");
    advertising_start();

    // Enter main loop.
    NRF_LOG_INFO("Entering main loop...");
    //Bring CS high to start
    nrf_gpio_pin_set(NRFX_SPIM_SS_PIN);
    for (;;)
    {
        //idle_state_handle();
        if(command_rdy) 
        {
          command_rdy = 0;
          process_rx_buffer();
        }
      // Was there a failed ble transfer? If so, try sending the buffer again
      if ((spi_cmd_flag == SPI_STREAM) & (ble_ready != NRF_SUCCESS))
      {
        NRF_LOG_INFO("Failed packet send");
        ble_ready = ble_nus_data_send(&m_nus, ble_tx_buf_send, &ble_packet_length, m_conn_handle);
        /*
        if (ble_ready == NRF_ERROR_INVALID_STATE || NRF_ERROR_RESOURCES)
        {
          APP_ERROR_CHECK(ble_ready);
        }
        */

      }
    }
}


/**
 * @}
 */
