/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) Initiator/Responder

*           Notes at the end of this file are left over from Decawave.
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



#define MAX_NETWORK_SIZE 32
#define OUT_OF_RANGE_LIMIT 10
#define DISTANCE_OFFSET 0.5f //should be tested more, this is an approximation for close ranging

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 200
#define RESP_MSG_TS_LEN 4 //timestamp length

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547


/* Messaging info */
static bool joined = FALSE;
static uint8 fctrl[] = {0x41, 0x88}; //default IEEE
static uint8 panid[] = {0xCA, 0xDE};
static uint8 local_dev_addr[2];
static uint8 session_id; //unused
static uint8 cluster_flag; //unused
static uint8 superframe_num; //unused
static uint32 seat_map = 0x00000001;
uint8 local_seat_num = 0; //change to something != 0 to program as a responder
// uint8 local_seat_num = 0xff;
static int local_x_pos = 0;
static int local_y_pos = 4;
static int local_z_pos = 0;

static uint8 init_addr[2]; //local copy of initiator address
static uint8 tx_dest_addr[2]; 
static uint8 rx_seat_num;
static uint8 num_network_devices = 1;

uwb_device_t in_network_devices[MAX_NETWORK_SIZE] = {NULL};

/* Timestamp variables*/
uint32 poll_tx_ts;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Frame sequence number, incremented after each initiator beacon transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define MAX_RX_BUF_LEN 127
static uint8 rx_buffer[MAX_RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Computed time of flight and distance */
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

/* TX Message Buffers*/
static uint8 beacon_msg[sizeof(message_header_t)+sizeof(beacon_payload_t)+2] = {0};
static uint8 join_req_msg[sizeof(message_header_t)+sizeof(join_req_payload_t)+2] = {0};
static uint8 join_conf_msg[sizeof(message_header_t)+sizeof(join_conf_payload_t)+2] = {0};

/* RX Message Pointers*/
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
  if (local_seat_num == 0)
  {
    /* Initiate ranging exchanges. */

    /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
    create_message(BEACON);
    dwt_writetxdata(sizeof(beacon_msg), beacon_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(beacon_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
    * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    /* Poll status register */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS))) { };

    poll_tx_ts = dwt_readtxtimestamplo32();

    tx_count++;
    frame_seq_nb++;
  }
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
  if (local_seat_num != 0)
  {
    dwt_rxreset();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }
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
  if (local_seat_num != 0)
  {
    dwt_rxreset();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn tx_conf_cb()
*
* @brief Callback to process TX confirmation events. This callback is unused as the TX interrupt has been disabled in the initialization process in main.c
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
  while (true)
  {
    ss_init_run();
    if (local_seat_num == 0)
    {
      float reception_rate = (float) rx_count / (float) tx_count * 100 / (num_network_devices-1);
      if (num_network_devices > MAX_NETWORK_SIZE/2)
      {
        if ((frame_seq_nb % 2) == 0)
        {
          printf("Reception rate # : %f\r\n",reception_rate);
          printf("seat_map = %08X\r\n", seat_map);
        }
      }
      else
      {
        printf("Reception rate # : %f\r\n",reception_rate);
        printf("seat_map = %08X\r\n", seat_map);
      }
      for (uint8 i = 1; i<MAX_NETWORK_SIZE; i++)
      {
        if ((seat_map & (1U << i)) != 0) 
        {
          if(in_network_devices[i].timeout_ctr == 0) //if device has not responded to several beacons
          {
            seat_map = seat_map ^ (1U << i); //remove from seat_map
            num_network_devices--;
            printf("SEAT %d unresponsive. Removing from network.\r\n", i);
          }
          else
          {
            in_network_devices[i].timeout_ctr--;
            if (num_network_devices > MAX_NETWORK_SIZE/2)
            {
              if ((frame_seq_nb % 2) == 0)
              {
                if (i < MAX_NETWORK_SIZE/2)
                {
                  printf("SEAT %d: %f, (%d, %d, %d)\r\n", in_network_devices[i].seat_num, in_network_devices[i].distance, in_network_devices[i].x_pos, in_network_devices[i].y_pos, in_network_devices[i].z_pos);
                }
              }
              else
              {
                if (i >= MAX_NETWORK_SIZE/2)
                {
                  printf("SEAT %d: %f, (%d, %d, %d)\r\n", in_network_devices[i].seat_num, in_network_devices[i].distance, in_network_devices[i].x_pos, in_network_devices[i].y_pos, in_network_devices[i].z_pos);
                }
              }
            }
            else
            {
              printf("SEAT %d: %f, (%d, %d, %d)\r\n", in_network_devices[i].seat_num, in_network_devices[i].distance, in_network_devices[i].x_pos, in_network_devices[i].y_pos, in_network_devices[i].z_pos);
            }
          }
        }
      }
    }
    vTaskDelay(RNG_DELAY_MS);
  }
}

void set_local_dev_addr()
{
  uint32 unique_id = dwt_getpartid();
  for (int i = 0; i < 2; i++)
  {
    local_dev_addr[i] = (unique_id >> (i*8)) & 0xFF; //extract lower 16 bits
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
      if(local_seat_num == 0) //if I am the initiator
      {
        rx_count++;
        uint8 rx_seat_num = rx_beacon_payload->seat_num;
        in_network_devices[rx_seat_num].timeout_ctr = OUT_OF_RANGE_LIMIT;
        uint32 resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32 rtd_init, rtd_resp;
        float clockOffsetRatio ;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 4 below. */
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
        distance = tof * SPEED_OF_LIGHT - DISTANCE_OFFSET;
        if (distance < 0)
        {
          distance = 0;
        }
        in_network_devices[rx_seat_num].distance = distance;
        in_network_devices[rx_seat_num].x_pos = rx_beacon_payload->x_pos;
        in_network_devices[rx_seat_num].y_pos = rx_beacon_payload->y_pos;
        in_network_devices[rx_seat_num].z_pos = rx_beacon_payload->z_pos;
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      }
      else //if I am not the initiator for this exchange
      {
        uint8 rx_seat_num = rx_beacon_payload->seat_num;
        if (rx_seat_num == local_seat_num)
        {
          joined = FALSE;
        }
        
        if (rx_seat_num == 0) //only the initiator can hold seat 0, so update local network data to match
        {
          poll_rx_ts = get_rx_timestamp_u64();
          frame_seq_nb = rx_header->seq_nb;
          session_id = rx_beacon_payload->session_id;
          cluster_flag = rx_beacon_payload->cluster_flag;
          superframe_num = rx_beacon_payload->superframe_num;
          seat_map = rx_beacon_payload->seat_map;
          init_addr[0] = rx_header->src_addr[0];
          init_addr[1] = rx_header->src_addr[1];

          if (seat_map == 1) //if the initiator has been restarted and has an empty seat map, reset 'joined' status
          {
            joined = FALSE;
          }

          num_network_devices = 0;
          uint32 copy_seat_map = seat_map;
          while(copy_seat_map) //update num_network_devices
          {
            num_network_devices += copy_seat_map & 0x1;
            copy_seat_map >>= 1;
          }

          if (joined==TRUE) //if already part of network respond accordingly
          {
            uint32 resp_tx_time;
            uint32 timeslot_delay;
            if (num_network_devices > MAX_NETWORK_SIZE/2)
            {
              if (((frame_seq_nb % 2) == 0) && (local_seat_num < MAX_NETWORK_SIZE/2))
              {
                timeslot_delay = local_seat_num*2;

                /* Compute final message transmission time. See NOTE 7 below. */
                resp_tx_time = (poll_rx_ts + ((1000*timeslot_delay) * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);

                /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
                resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
                
                create_message(BEACON);
                send_message(beacon_msg, sizeof(beacon_msg));                
              }
              else if (((frame_seq_nb % 2) == 1) && (local_seat_num >= MAX_NETWORK_SIZE/2))
              {
                timeslot_delay = (local_seat_num - MAX_NETWORK_SIZE/2 + 1)*2;

                /* Compute final message transmission time. See NOTE 7 below. */
                resp_tx_time = (poll_rx_ts + ((1000*timeslot_delay) * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(resp_tx_time);

                /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
                resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
                
                create_message(BEACON);
                send_message(beacon_msg, sizeof(beacon_msg));
              }
              else
              {
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
              }
            }
            else
            {
              timeslot_delay = local_seat_num*2;

              /* Compute final message transmission time. See NOTE 7 below. */
              resp_tx_time = (poll_rx_ts + ((1000*timeslot_delay) * UUS_TO_DWT_TIME)) >> 8;
              dwt_setdelayedtrxtime(resp_tx_time);

              /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
              resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
              
              create_message(BEACON);
              send_message(beacon_msg, sizeof(beacon_msg));
            }
          }
          else //if not part of network make network join request if there is space
          {
            uint32 inverted_map = ~(rx_beacon_payload->seat_map);
            if (inverted_map == 0) //all seats are full
            {
              dwt_rxenable(DWT_START_RX_IMMEDIATE);
              return; //exit and wait until there is an opening
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
              local_seat_num = open_seat; //set local seat number to lowest open slot for join request

              create_message(JOIN_REQ);
              dwt_writetxdata(sizeof(join_req_msg), join_req_msg, 0); /* Zero offset in TX buffer. See Note 5 below.*/
              dwt_writetxfctrl(sizeof(join_req_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
              
              tx_timer_period = 65;
              xSemaphoreGive(xSendMsgSemaphore);
              printf("Network join request initiated for SEAT %d.\r\n", local_seat_num);
            }
          }
        }
        else
        {
          dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }        
      }
      break;
    case (JOIN_REQ):
      if (local_seat_num == 0)
      {
        rx_join_req_payload = (join_req_payload_t *)rx_payload;
        uint32 inverted_map = ~seat_map;
        if ((seat_map & (1U << (rx_join_req_payload->seat_num))) == 0)
        {
          tx_dest_addr[0] = rx_header->src_addr[0];
          tx_dest_addr[1] = rx_header->src_addr[1];
          rx_seat_num = rx_join_req_payload->seat_num;
          create_message(JOIN_CONF);
          dwt_writetxdata(sizeof(join_conf_msg), join_conf_msg, 0); /* Zero offset in TX buffer. */
          dwt_writetxfctrl(sizeof(join_conf_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
          printf("Responding to %d request for SEAT %d.\r\n", tx_dest_addr[0], rx_seat_num);
          seat_map = seat_map | (1U << (rx_join_req_payload->seat_num)); //update seat map to reflect occupation of slot
          num_network_devices++;
          in_network_devices[rx_seat_num].dev_addr[0] = tx_dest_addr[0];
          in_network_devices[rx_seat_num].dev_addr[1] = tx_dest_addr[1];
          in_network_devices[rx_seat_num].seat_num = rx_seat_num;
          in_network_devices[rx_seat_num].timeout_ctr = OUT_OF_RANGE_LIMIT;
          tx_timer_period = 10;
          xSemaphoreGive(xSendMsgSemaphore);
        }
      }
      else
      {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      }
      break;
    case (JOIN_CONF):
      if ((rx_header->dest_addr[0]==local_dev_addr[0]) && (rx_header->dest_addr[1]==local_dev_addr[1]))
      {
        rx_join_conf_payload = (join_conf_payload_t *)rx_payload;
        local_seat_num = rx_join_conf_payload->seat_num;
        joined = TRUE;
        printf("Joined network successfully at SEAT %d\r\n", local_seat_num);\
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      }
      else
      {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
      }
      break;
    default:
      printf("Invalid message type received.\r\n");
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
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
    local_dev_addr[0], local_dev_addr[1],
    (uint8)msg_type
  };

  switch (msg_type)
  {
    case (BEACON):
      tx_msg_hdr.dest_addr[0] = 0xFF; //beacons are addressed to all transceivers (0xFFFF)
      tx_msg_hdr.dest_addr[1] = 0xFF;

      beacon_payload_t beacon_payload = {
        session_id,
        cluster_flag,
        superframe_num,
        local_seat_num,
        seat_map,
        0, 0, 0, 0, //rx timestamp
        0, 0, 0, 0, //tx timestamp
        local_x_pos,
        local_y_pos,
        local_z_pos
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
      join_req_payload_t join_req_payload = {local_seat_num};
      
      memcpy(join_req_msg, &tx_msg_hdr, sizeof(message_header_t));
      memcpy(join_req_msg + sizeof(message_header_t), &join_req_payload, sizeof(join_req_payload_t));
      break;
    case (JOIN_CONF):
      tx_msg_hdr.dest_addr[0] = tx_dest_addr[0];
      tx_msg_hdr.dest_addr[1] = tx_dest_addr[1];
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
      ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
      if (ret == DWT_SUCCESS)
      {
        /* Poll DW1000 until TX frame sent event set. See NOTE 5 below. */
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        {};

        /* Clear TXFRS event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
      }
      else
      {
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
      }
    }
  }
}

void send_message(uint8* out_message, uint8 message_size)
{
  int ret;
  dwt_writetxdata(message_size, out_message, 0); /* Zero offset in TX buffer. See Note 5 below.*/
  dwt_writetxfctrl(message_size, 0, 1); /* Zero offset in TX buffer, ranging. */
  ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

  /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
  if (ret == DWT_SUCCESS)
  {
    /* Poll DW1000 until TX frame sent event set. See NOTE 5 below. */
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
    {};

    /* Clear TXFRS event. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
  }
  else
  {
    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
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
