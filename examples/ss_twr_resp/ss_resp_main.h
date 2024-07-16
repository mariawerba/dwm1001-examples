 /*----------------------------------------------------------------------------
 *  @file    ss_resp_main.h
 *  @brief   Single-sided two-way ranging (SS TWR) responder with interrupt example code -- Header file
 *
 * 
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

/* Declaration of static functions. */
int ss_resp_run(void);

void rx_ok_cb(const dwt_cb_data_t *cb_data);

void rx_to_cb(const dwt_cb_data_t *cb_data);

void rx_err_cb(const dwt_cb_data_t *cb_data);

void tx_conf_cb(const dwt_cb_data_t *cb_data);

static uint64 get_rx_timestamp_u64(void);

static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);

void ss_responder_task_function (void * pvParameter);

void set_src_addr();

void create_tx_timer();