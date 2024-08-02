/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code
*
*           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
*           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
*           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
*           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
*           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
*
*
*           Notes at the end of this file, expand on the inline comments.
* 
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "ss_init_main.h"
#include "semphr.h"
#include "uwb_messages.h"

#define APP_NAME "SS TWR INIT v1.3"

/*Import common messaging functions*/

/* Messaging info */
static bool joined = FALSE;
static uint8 fctrl[] = {0x41, 0x88}; //default, means something in IEEE
static uint8 panid[] = {0xCA, 0xDE}; //default
static uint8 src_addr[2];
static uint8 session_id;
static uint8 cluster_flag;
static uint8 superframe_num;
static uint8 seat_num = 0;
static uint32 seat_map = 0x00000001;
static uint8 init_addr[2];

static uint8 rx_addr[2];
static uint8 rx_seat_num;

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 250

/* Frames used in the ranging process. See NOTE 1,2 below. */
//static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xff, 0xff, 0xff, 0xff, 0x4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define MAX_RX_BUF_LEN 127
static uint8 rx_buffer[MAX_RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);
static void resp_msg_get_rx_ts(beacon_payload_t *tx_payload, uint32 *ts);
static void resp_msg_get_tx_ts(beacon_payload_t *tx_payload, uint32 *ts);

/*Interrupt flag*/
static volatile int tx_int_flag = 0 ; // Transmit success interrupt flag
static volatile int rx_int_flag = 0 ; // Receive success interrupt flag
static volatile int to_int_flag = 0 ; // Timeout interrupt flag
static volatile int er_int_flag = 0 ; // Error interrupt flag 

/*Transactions Counters */
static volatile int tx_count = 0 ; // Successful transmit counter
static volatile int rx_count = 0 ; // Successful receive counter

/*Message Buffers*/
static uint8 beacon_msg[sizeof(message_header_t)+sizeof(beacon_payload_t)+2] = {0};
static uint8 join_req_msg[sizeof(message_header_t)+sizeof(join_req_payload_t)+2] = {0};
static uint8 join_conf_msg[sizeof(message_header_t)+sizeof(join_conf_payload_t)+2] = {0};

static message_header_t *rx_header;
static uint8 *rx_payload;
static beacon_payload_t *rx_beacon_payload;
static join_req_payload_t *rx_join_req_payload;
static join_conf_payload_t *rx_join_conf_payload;

/*Added FreeRTOS elements*/
SemaphoreHandle_t xProcessMsgSemaphore;
SemaphoreHandle_t xSendMsgSemaphore;
TaskHandle_t process_message_handle;
TaskHandle_t tx_timer_handle;
void send_timed_message(void * pvParameter);
static uint32 tx_timer_period = 100; //set to 100ms as default

/*Responder constants*/
// Not enough time to write the data so TX timeout extended for nRF operation.
// Might be able to get away with 800 uSec but would have to test
// See note 6 at the end of this file
#define POLL_RX_TO_RESP_TX_DLY_UUS  1100

/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500


/*! ------------------------------------------------------------------------------------------------------------------
* @fn main()
*
* @brief Application entry point.
*
* @param  none
*
* @return none
*/
int ss_init_run(void)
{
  /* Loop forever initiating ranging exchanges. */


  /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  
  create_message(BEACON);
  dwt_writetxdata(sizeof(beacon_msg), beacon_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(beacon_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
  // dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
  // dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */


  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
  * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  /*Waiting for transmission success flag*/

  if (1)
  {
    tx_count++;
    printf("Transmission # : %d\r\n",tx_count);

    /*Reseting tx interrupt flag*/
    tx_int_flag = 0 ;
  }

  /* Wait for reception, timeout or error interrupt flag*/
  while (!(rx_int_flag || to_int_flag|| er_int_flag))
  {};

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (to_int_flag || er_int_flag)
  {
    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();

    /*Reseting interrupt flag*/
    to_int_flag = 0 ;
    er_int_flag = 0 ;
  }

    /* Execute a delay between ranging exchanges. */
  // deca_sleep(RNG_DELAY_MS);
  // return(1);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_ok_cb()
*
* @brief Callback to process RX good frame events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  rx_int_flag = 1 ;
  xSemaphoreGiveFromISR(xProcessMsgSemaphore, NULL);
  /* TESTING BREAKPOINT LOCATION #1 */
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_to_cb()
*
* @brief Callback to process RX timeout events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_to_cb(const dwt_cb_data_t *cb_data)
{
  to_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #2 */
  printf("TimeOut\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_err_cb()
*
* @brief Callback to process RX error events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_err_cb(const dwt_cb_data_t *cb_data)
{
  er_int_flag = 1 ;
  /* TESTING BREAKPOINT LOCATION #3 */
  printf("Transmission Error : may receive package from different UWB device\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn tx_conf_cb()
*
* @brief Callback to process TX confirmation events
*
* @param  cb_data  callback data
*
* @return  none
*/
void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
  /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
  * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
  * by DW1000 using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
  * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
  * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

  tx_int_flag = 1 ;
  //printf("seat map = %d\n\r", seat_map);
  /* TESTING BREAKPOINT LOCATION #4 */
}


/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
*        least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to get
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
  *ts += ts_field[i] << (i * 8);
  }
}

static void resp_msg_get_rx_ts(beacon_payload_t *tx_payload, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    *ts += (uint8)(tx_payload->rx_ts[i]) << (i * 8);
  }
}

static void resp_msg_get_tx_ts(beacon_payload_t *tx_payload, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    *ts += (uint8)(tx_payload->tx_ts[i]) << (i * 8);
  }
}

static void resp_msg_set_rx_ts(beacon_payload_t *tx_payload, const uint64 ts)
{
  int i;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    tx_payload->rx_ts[i] = (ts >> ((i) * 8)) & 0xFF;
  }
}

static void resp_msg_set_tx_ts(beacon_payload_t *tx_payload, const uint64 ts)
{
  int i;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    tx_payload->tx_ts[i] = (ts >> ((i) * 8)) & 0xFF;
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
*
* @brief Get the RX time-stamp in a 64-bit variable.
*        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
*
* @param  none
*
* @return  64-bit value of the read time-stamp.
*/
static uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}


/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_initiator_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  // dwt_setleds(DWT_LEDS_ENABLE);

  while (true)
  {
    ss_init_run();
    /* Delay a task for a given number of ticks */
    vTaskDelay(RNG_DELAY_MS);
    /* Tasks must be implemented to never return... */
  }
}

void set_src_addr()
{
  uint32 unique_id = dwt_getpartid();
  for (int i = 0; i < 2; i++)
  {
    src_addr[i] = (unique_id >> (i*8)) & 0xFF; //extract lower 16 bits
  }
}

void create_tasks_and_resources()
{
  xProcessMsgSemaphore = xSemaphoreCreateBinary();
  xSendMsgSemaphore = xSemaphoreCreateBinary();

  if (xProcessMsgSemaphore != NULL)
  {
    xTaskCreate(process_message, "proc_msg", configMINIMAL_STACK_SIZE + 2000, NULL, 2, &process_message_handle);
  }

  if (xProcessMsgSemaphore != NULL)
  {
    xTaskCreate(send_timed_message, "send_timed_msg", configMINIMAL_STACK_SIZE + 2000, NULL, 2, &tx_timer_handle);
  }
}

void process_message(void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  while (true)
  {
    if(xSemaphoreTake(xProcessMsgSemaphore, portMAX_DELAY) == pdTRUE)
    {
      uint32 frame_len;

      /* A frame has been received, read it into the local buffer. */
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
      if (frame_len <= MAX_RX_BUF_LEN)
      {
        dwt_readrxdata(rx_buffer, frame_len, 0);
        process_rx_buffer(rx_buffer, frame_len);
      }
      rx_int_flag = 0; 

      // /* Check that the frame is the expected response from the companion "SS TWR responder" example.
      // * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
      // rx_buffer[ALL_MSG_SN_IDX] = 0;
      // if (TRUE)//(memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
      // {	
      //   rx_count++;
      //   printf("Reception # : %d\r\n",rx_count);
      //   float reception_rate = (float) rx_count / (float) tx_count * 100;
      //   printf("Reception rate # : %f\r\n",reception_rate);
      //   uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
      //   int32 rtd_init, rtd_resp;
      //   float clockOffsetRatio ;

      //   /* Retrieve poll transmission and response reception timestamps. See NOTE 4 below. */
      //   poll_tx_ts = dwt_readtxtimestamplo32();
      //   resp_rx_ts = dwt_readrxtimestamplo32();

      //   /* Read carrier integrator value and calculate clock offset ratio. See NOTE 6 below. */
      //   clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;

      //   /* Get timestamps embedded in response message. */
      //   // resp_msg_get_ts(&rx_buffer[15], &poll_rx_ts);
      //   //resp_msg_get_ts(&rx_buffer[19], &resp_tx_ts);
      //   message_header_t *rx_header = (message_header_t *) rx_buffer;
      //   if (rx_header->msg_type == JOIN_REQ)
      //   {
      //     join_req_payload_t *rx_payload = (join_req_payload_t *) (rx_buffer + sizeof(*rx_header));
      //     uint8 inverted_map = ~seat_map;
      //     if ((inverted_map & (rx_payload->seat_num)) != 0)
      //     {

      //         //send
      //     }
      //   }
      //   if(rx_header->msg_type == BEACON)
      //   {
      //     beacon_payload_t *rx_payload = (beacon_payload_t *) (rx_buffer + sizeof(*rx_header));
      //     resp_msg_get_rx_ts(rx_payload, &poll_rx_ts);
      //     resp_msg_get_tx_ts(rx_payload, &resp_tx_ts);
      //   }


      //   /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
      //   rtd_init = resp_rx_ts - poll_tx_ts;
      //   rtd_resp = resp_tx_ts - poll_rx_ts;

      //   tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to clear warning 
      //   distance = tof * SPEED_OF_LIGHT;
      //   printf("Distance : %f\r\n",distance);

      //   /*Reseting receive interrupt flag*/
      //   rx_int_flag = 0; 
      // }
    }
  }
}


void process_rx_buffer(uint8 *rx_buf, uint16 rx_data_len)
{
  rx_header = (message_header_t *) rx_buf;
  rx_payload = rx_buf + sizeof(message_header_t);

  switch (rx_header->msg_type)
  {
    case (BEACON):
      rx_beacon_payload = (beacon_payload_t *)rx_payload;
      if(seat_num == 0) //if I am the initiator
      {
        rx_count++;
        printf("Reception # : %d\r\n",rx_count);
        float reception_rate = (float) rx_count / (float) tx_count * 100;
        printf("Reception rate # : %f\r\n",reception_rate);
        uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32 rtd_init, rtd_resp;
        float clockOffsetRatio ;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 4 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 6 below. */
        clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;

        /* Get timestamps embedded in response message. */
        resp_msg_get_rx_ts(rx_payload, &poll_rx_ts);
        resp_msg_get_tx_ts(rx_payload, &resp_tx_ts);

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to clear warning 
        distance = tof * SPEED_OF_LIGHT;
        printf("Distance : %f\r\n",distance);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      }
      else
      {
        uint8 rx_seat_num = rx_beacon_payload->seat_num;
        if (rx_seat_num == 0) //only the initiator can hold seat 0, so update local network data to match
        {
          session_id = rx_beacon_payload->session_id;
          cluster_flag = rx_beacon_payload->cluster_flag;
          superframe_num = rx_beacon_payload->superframe_num;
          seat_map = rx_beacon_payload->seat_map;
          init_addr[0] = rx_header->src_addr[0];
          init_addr[1] = rx_header->src_addr[1];

          if (joined==TRUE) //if already part of network respond accordingly
          {
            create_message(BEACON);
            
            dwt_writetxdata((sizeof(message_header_t)+sizeof(beacon_payload_t)+2), beacon_msg, 0); /* Zero offset in TX buffer. See Note 5 below.*/
            dwt_writetxfctrl((sizeof(message_header_t)+sizeof(beacon_payload_t)+2), 0, 1); /* Zero offset in TX buffer, ranging. */
            
            tx_timer_period = 10*(seat_num);
            if (tx_timer_handle != NULL) 
            {
              xTimerChangePeriod(tx_timer_handle, pdMS_TO_TICKS(tx_timer_period), 0);
              xTimerStart(tx_timer_handle, 0);
              printf("Tx timer started.\r\n");
              // BaseType_t result = xTimerStart(tx_timer_handle, 0);
              // if (result != pdPASS) 
              // {
              //     printf("Tx timer start failed.\r\n");
              // }
            } else { printf("Tx timer start failed.\r\n"); }
          }
          else //if not part of network make network join request if there is space
          {
            uint32 inverted_map = ~(rx_beacon_payload->seat_map);
            if (inverted_map == 0) //all seats are full
            {
              return; //have to wait until there is an opening
            }
            else
            {
              uint8 open_seat = 0;
              uint32 mask = 0x1;
              while ((inverted_map & mask) == 0)
              {
                mask <<= 1;
                open_seat++;
              }
              seat_num = open_seat; //set local seat number to lowest open slot for jjoin request

              create_message(JOIN_REQ);

              dwt_writetxdata((sizeof(message_header_t)+sizeof(join_req_payload_t)+2), beacon_msg, 0); /* Zero offset in TX buffer. See Note 5 below.*/
              dwt_writetxfctrl((sizeof(message_header_t)+sizeof(join_req_payload_t)+2), 0, 1); /* Zero offset in TX buffer, ranging. */
              
              tx_timer_period = 100;
              if (tx_timer_handle != NULL) 
              {
                xTimerChangePeriod(tx_timer_handle, pdMS_TO_TICKS(tx_timer_period), 0);
                xTimerStart(tx_timer_handle, 0);
                printf("Tx timer started.\r\n");
              } else { printf("Tx timer start failed.\r\n"); }
            }

            printf("Requesting network join %d.\r\n", inverted_map);
          }
        }
      }
      break;
    case (JOIN_REQ):
      if (seat_num == 0)
      {
        rx_join_req_payload = (join_req_payload_t *)rx_payload;
        uint32 inverted_map = ~seat_map;
        if ((seat_map & (1U << (rx_join_req_payload->seat_num))) == 0)
        {
          rx_addr[0] = rx_header->src_addr[0];
          rx_addr[1] = rx_header->src_addr[1];
          rx_seat_num = rx_join_req_payload->seat_num;
          create_message(JOIN_CONF);
          dwt_writetxdata(sizeof(join_conf_msg), join_conf_msg, 0); /* Zero offset in TX buffer. */
          dwt_writetxfctrl(sizeof(join_conf_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
          // dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
          printf("Responding to %d request for SEAT %d.\r\n", rx_addr[0], rx_seat_num);
          seat_map = seat_map | (1U << (rx_join_req_payload->seat_num)); //update seat map to reflect occupation of slot
          tx_timer_period = 40;
          xSemaphoreGive(xSendMsgSemaphore);
            //send
        }
      //send_message(beacon_msg, (sizeof(message_header_t)+sizeof(beacon_payload_t)+2));
      }
      break;
    case (JOIN_CONF):
      //insert join conf process here
      break;
    default:
      printf("Invalid message type received.\r\n");
      break;
  }
}

void create_message(message_type_t msg_type)
{
  message_header_t tx_msg_hdr = {
    fctrl[0], fctrl[1],
    frame_seq_nb,
    panid[0], panid[1],
    0, 0, // these destination address fields will be set below
    src_addr[0], src_addr[1],
    (uint8)msg_type
  };

  //TODO: move this to its own function?
  /* Get timestamp info*************************************/
  uint32 resp_tx_time;

  /* Retrieve poll reception timestamp. */
  poll_rx_ts = get_rx_timestamp_u64();

  /* Compute final message transmission time. See NOTE 7 below. */
  resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
  dwt_setdelayedtrxtime(resp_tx_time);

  /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
  resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
  /**********************************************************/

  switch (msg_type)
  {
    case (BEACON):
      tx_msg_hdr.dest_addr[0] = 0xFF; //beacons are addressed to all transceivers (0xFFFF)
      tx_msg_hdr.dest_addr[1] = 0xFF;

      beacon_payload_t beacon_payload = {
        session_id,
        cluster_flag,
        superframe_num,
        seat_num,
        seat_map,
        0, 0, 0, 0, //rx timestamp
        0, 0, 0, 0, //tx timestamp
        0,          //data_type
        0, 0, 0, 0, //data1
        0, 0, 0, 0  //data2
      };
      /* Write all timestamps in the final message. See NOTE 8 below. */
      resp_msg_set_rx_ts(&beacon_payload, poll_rx_ts);
      resp_msg_set_tx_ts(&beacon_payload, resp_tx_ts);

      memcpy(beacon_msg, &tx_msg_hdr, sizeof(message_header_t));
      memcpy(beacon_msg + sizeof(message_header_t), &beacon_payload, sizeof(beacon_payload_t));
      break;
    case (JOIN_REQ):
      tx_msg_hdr.dest_addr[0] = init_addr[0]; //this is the initiators address right now but it shouldn't make a difference
      tx_msg_hdr.dest_addr[1] = init_addr[1];
      join_req_payload_t join_req_payload = {seat_num};
      
      memcpy(join_req_msg, &tx_msg_hdr, sizeof(message_header_t));
      memcpy(join_req_msg + sizeof(message_header_t), &join_req_payload, sizeof(join_req_payload_t));
      break;
    case (JOIN_CONF):
      tx_msg_hdr.dest_addr[0] = rx_addr[0]; //this is the initiators address right now but it shouldn't make a difference
      tx_msg_hdr.dest_addr[1] = rx_addr[1];
      join_conf_payload_t join_conf_payload = {rx_seat_num};
      
      memcpy(join_conf_msg, &tx_msg_hdr, sizeof(message_header_t));
      memcpy(join_conf_msg + sizeof(message_header_t), &join_conf_payload, sizeof(join_req_payload_t));
      break;
    default:
      printf("Cannot create invalid message type.\r\n");
      break;
  }
}




void send_timed_message(void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  while (true)
  {
    if(xSemaphoreTake(xSendMsgSemaphore, portMAX_DELAY) == pdTRUE)
    {
      vTaskDelay(tx_timer_period);
      int ret;
      // dwt_writetxdata(message_size, out_message, 0); /* Zero offset in TX buffer. See Note 5 below.*/
      // dwt_writetxfctrl(message_size, 0, 1); /* Zero offset in TX buffer, ranging. */
      //ret = dwt_starttx(DWT_START_TX_DELAYED);

      ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
      if (ret == DWT_SUCCESS)
      {
        /* Poll DW1000 until TX frame sent event set. See NOTE 5 below. */
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        {};

        /* Clear TXFRS event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        /* Increment frame sequence number after transmission of the poll message (modulo 256). */
        frame_seq_nb++;
      }
      else
      {
        /* If we end up in here then we have not succeded in transmitting the packet we sent up.
        POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for porting to different processors.
        For slower platforms where the SPI is at a slower speed or the processor is operating at a lower
        frequency (Comparing to STM32F, SPI of 18MHz and Processor internal 72MHz)this value needs to be increased.
        Knowing the exact time when the responder is going to send its response is vital for time of flight
        calculation. The specification of the time of respnse must allow the processor enough time to do its
        calculations and put the packet in the Tx buffer. So more time is required for a slower system(processor).
        */

        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
      }

      //xTimerStop(tx_timer_handle, 0);
      //xTimerChangePeriod(tx_timer_handle, pdMS_TO_TICKS(tx_timer_period), 0);
    }
  }
}


/*****************************************************************************************************************************************************
* NOTES:
*
* 1. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 2 below.
*     - byte 7/8: source address, see NOTE 2 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 2. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 3. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 4. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 5. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 6. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
*
****************************************************************************************************************************************************/
