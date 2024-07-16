/*! ----------------------------------------------------------------------------
*  @file    ss_resp_main.c
*  @brief   Single-sided two-way ranging (SS TWR) responder example code
*
*           This is a simple code example which acts as the responder in a SS TWR distance measurement exchange.
*           This application waits for a "poll" message (recording the RX time-stamp of the poll) expected from
*           the "SS TWR initiator" example code (companion to this application), and
*           then sends a "response" message recording its TX time-stamp.
*
*           Notes at the end of this file, to expand on the inline comments.
*
* @attention
*
* Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include "sdk_config.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "uwb_messages.h"


/* Define constants *******************************************************************/
#define RESP_MSG_TS_LEN 4
#define RX_BUF_LEN 127 //longest std frame length

/* Inter-ranging delay period, in milliseconds. See NOTE 1*/
#define RNG_DELAY_MS 80

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

// Not enough time to write the data so TX timeout extended for nRF operation.
// Might be able to get away with 800 uSec but would have to test
// See note 6 at the end of this file
#define POLL_RX_TO_RESP_TX_DLY_UUS  1100

/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/**************************************************************************************/


/* Messaging info */
static bool joined = FALSE;
static uint8 fctrl[] = {0x41, 0x88}; //default, means something in IEEE
static uint8 panid[] = {0xCA, 0xDE}; //default
static uint8 src_addr[2];
static uint8 session_id;
static uint8 cluster_flag;
static uint8 superframe_num;
static uint8 seat_num = 0xFF;
static uint8 seat_map;
static uint8 init_addr[2];


/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Declaration of static functions. */
//static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);
static void resp_msg_set_rx_ts(beacon_payload_t *tx_payload, const uint64 ts);
static void resp_msg_set_tx_ts(beacon_payload_t *tx_payload, const uint64 ts);
//static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

/*Interrupt flag*/
static volatile int tx_int_flag = 0 ; // Transmit success interrupt flag
static volatile int rx_int_flag = 0 ; // Receive success interrupt flag
static volatile int to_int_flag = 0 ; // Timeout interrupt flag
static volatile int er_int_flag = 0 ; // Error interrupt flag

/*Transactions Counters */
static volatile int tx_count = 0 ; // Successful transmit counter
static volatile int rx_count = 0 ; // Successful receive counter

void create_message(message_type_t msg_type);
void send_timed_message(TimerHandle_t tx_timer_handle);
void send_message(uint8* out_message, uint8 message_size);

static uint8 beacon_msg[sizeof(message_header_t)+sizeof(beacon_payload_t)+2] = {0};
static uint8 join_req_msg[sizeof(message_header_t)+sizeof(join_req_payload_t)+2] = {0};
static uint8 join_conf_msg[sizeof(message_header_t)+sizeof(join_conf_payload_t)+2] = {0};

TimerHandle_t tx_timer_handle;
static uint32 timer_period = 100; //set to 100ms as default


/*! ------------------------------------------------------------------------------------------------------------------
* @fn main()
*
* @brief Application entry point.
*
* @param  none
*
* @return none
*/

int ss_resp_run(void)
{
  //maybe fill with something eventually, but for now do nothing
  return(1);    
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
  /* TESTING BREAKPOINT LOCATION #1 */
  dwt_forcetrxoff();
  //ss_resp_run();
  //dwt_rxreset();

  /* A frame has been received, copy it to our local buffer. See NOTE 6 below. */
  if (cb_data->datalength <= RX_BUF_LEN)
  {
      dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
      uint16 rx_data_length = cb_data->datalength;
      process_packet(rx_buffer, rx_data_length);
  }

  /* Check that the frame is a poll sent by "SS TWR initiator" example.
  * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
  // rx_buffer[ALL_MSG_SN_IDX] = 0;
 
  dwt_rxreset();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
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
  //printf("TimeOut\r\n");
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
  //printf("Transmission Error : may receive package from different UWB device\r\n");
  dwt_rxreset();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
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
  /* TESTING BREAKPOINT LOCATION #4 */
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn final_msg_set_ts()
*
* @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
*        response message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to fill
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
  int i;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
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



/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_responder_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  dwt_setleds(DWT_LEDS_ENABLE);

  while (true)
  {
    ss_resp_run();
    /* Delay a task for a given number of ticks */
    vTaskDelay(RNG_DELAY_MS);
    /* Tasks must be implemented to never return... */
  }
}

void process_packet (uint8 *rx_buf, uint16 rx_data_len)
{
  message_header_t *rx_header = (message_header_t *) rx_buf;

  switch (rx_header->msg_type)
  {
    case (5):
      // uint16 payload_size = rx_data_len - sizeof(*rx_header);
      // printf("payload size = %d\r\n", payload_size);
      printf('\r\n');
      beacon_payload_t *rx_payload = (beacon_payload_t *) (rx_buf + sizeof(*rx_header));
      uint8 rx_seat_num = rx_payload->seat_num;
      if (rx_seat_num == 0) //only the initiator can hold seat 0, so update local network data to match
      {
        session_id = rx_payload->session_id;
        cluster_flag = rx_payload->cluster_flag;
        superframe_num = rx_payload->superframe_num;
        seat_map = rx_payload->seat_map;
        init_addr[0] = rx_header->src_addr[0];
        init_addr[1] = rx_header->src_addr[1];

        if (joined==TRUE) //if already part of network respond accordingly
        {
          create_message(BEACON);
          
          dwt_writetxdata((sizeof(message_header_t)+sizeof(beacon_payload_t)+2), beacon_msg, 0); /* Zero offset in TX buffer. See Note 5 below.*/
          dwt_writetxfctrl((sizeof(message_header_t)+sizeof(beacon_payload_t)+2), 0, 1); /* Zero offset in TX buffer, ranging. */
          
          timer_period = 10*(seat_num);
          if (tx_timer_handle != NULL) 
          {
            xTimerChangePeriod(tx_timer_handle, pdMS_TO_TICKS(timer_period), 0);
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
          uint8 inverted_map = ~(rx_payload->seat_map);
          if (inverted_map == 0) //all seats are full
          {
            return; //have to wait until there is an opening
          }
          else
          {
            uint8 open_seat = 0;
            uint8 mask = 0x1;
            while ((inverted_map & mask) == 0)
            {
              mask <<= 1;
              open_seat++;
            }
            seat_num = open_seat; //set local seat number to lowest open slot for jjoin request

            create_message(JOIN_REQ);

            dwt_writetxdata((sizeof(message_header_t)+sizeof(join_req_payload_t)+2), beacon_msg, 0); /* Zero offset in TX buffer. See Note 5 below.*/
            dwt_writetxfctrl((sizeof(message_header_t)+sizeof(join_req_payload_t)+2), 0, 1); /* Zero offset in TX buffer, ranging. */
            
            timer_period = 100;
            if (tx_timer_handle != NULL) 
            {
              xTimerChangePeriod(tx_timer_handle, pdMS_TO_TICKS(timer_period), 0);
              xTimerStart(tx_timer_handle, 0);
              printf("Tx timer started.\r\n");
            } else { printf("Tx timer start failed.\r\n"); }
          }

          printf("Requesting network join %d.\r\n", inverted_map);
        }
      }
      
      break;
    case (BEACON):
      printf("just checking \r\n");
      create_message(BEACON);
      send_message(beacon_msg, (sizeof(message_header_t)+sizeof(beacon_payload_t)+2));
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

      //uint8 beacon_msg[sizeof(message_header_t)+sizeof(beacon_payload_t)+2] = {0};
      memcpy(beacon_msg, &tx_msg_hdr, sizeof(message_header_t));
      memcpy(beacon_msg + sizeof(message_header_t), &beacon_payload, sizeof(beacon_payload_t));
      break;
    case (JOIN_REQ):
      tx_msg_hdr.dest_addr[0] = init_addr[0]; //this is the initiators address right now but it shouldn't make a difference
      tx_msg_hdr.dest_addr[1] = init_addr[1];
     
      join_req_payload_t join_req_payload = {seat_num};
      //uint8 join_req_msg[sizeof(message_header_t)+sizeof(join_req_payload_t)+2] = {0};
      memcpy(join_req_msg, &tx_msg_hdr, sizeof(message_header_t));
      memcpy(join_req_msg + sizeof(message_header_t), &join_req_payload, sizeof(join_req_payload_t));
      break;
    case (JOIN_CONF):
    
      break;
    default:
      printf("Cannot create invalid message type.\r\n");
      break;
  }
}


void send_timed_message(TimerHandle_t tx_timer_handle)
{
  int ret;
  // dwt_writetxdata(message_size, out_message, 0); /* Zero offset in TX buffer. See Note 5 below.*/
  // dwt_writetxfctrl(message_size, 0, 1); /* Zero offset in TX buffer, ranging. */
  //ret = dwt_starttx(DWT_START_TX_DELAYED);

  ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

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

  xTimerStop(tx_timer_handle, 0);
  xTimerChangePeriod(tx_timer_handle, pdMS_TO_TICKS(timer_period), 0);
}


void send_message(uint8* out_message, uint8 message_size)
{
  int ret;
  dwt_writetxdata(message_size, out_message, 0); /* Zero offset in TX buffer. See Note 5 below.*/
  dwt_writetxfctrl(message_size, 0, 1); /* Zero offset in TX buffer, ranging. */
  ret = dwt_starttx(DWT_START_TX_DELAYED);

  //ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

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
}


void set_src_addr()
{
  uint32 unique_id = dwt_getpartid();
  for (int i = 0; i < 2; i++)
  {
    src_addr[i] = (unique_id >> (i*8)) & 0xFF; //extract lower 16 bits
  }
}

void create_tx_timer()
{
  tx_timer_handle = xTimerCreate("tx_timer_handle", pdMS_TO_TICKS(timer_period), pdTRUE, (void *)0, send_timed_message);
  if (tx_timer_handle == NULL) {
      printf("Tx timer creation failed.\r\n");
  }
}

/*****************************************************************************************************************************************************
* NOTES:
*
* 1. This is the task delay when using FreeRTOS. Task is delayed a given number of ticks. Useful to be able to define this out to see the effect of the RTOS
*    on timing.
* 2. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 3 below.
*     - byte 7/8: source address, see NOTE 3 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 4. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 5. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 6. POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for porting to different processors. For slower platforms where the SPI is at a slower speed
*    or the processor is operating at a lower frequency (Comparing to STM32F, SPI of 18MHz and Processor internal 72MHz)this value needs to be increased.
*    Knowing the exact time when the responder is going to send its response is vital for time of flight calculation. The specification of the time of
*    respnse must allow the processor enough time to do its calculations and put the packet in the Tx buffer. So more time required for a slower
*    system(processor).
* 7. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
*    register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
*    response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
*    lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
*    8 bits.
* 8. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
*    time-of-flight computation) can be handled by a 32-bit subtraction.
* 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
*10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*    DW1000 API Guide for more details on the DW1000 driver functions.
*
****************************************************************************************************************************************************/
 