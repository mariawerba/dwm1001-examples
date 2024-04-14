/*!
* @brief Component name:	UART
*
* Simple two-wire UART application level driver.
* Provides buffered UART interface, compatible with
* a redirected STDIO for printf and getc.
*
* @file UART.c
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "app_fifo.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_uart.h"
#include "UART.h"
#include "bsp.h"
#include "boards.h"

#define NO_PARITY	false

// UART circular buffers - Tx and Rx size
#define UART_TX_BUF_SIZE 512
#define UART_RX_BUF_SIZE 32

static uint8_t rx_buf[UART_RX_BUF_SIZE];              
static app_fifo_t m_rx_fifo; 
static bool uart_rx_data_ready = false;

// UART initialisation structure
const app_uart_comm_params_t comm_params =
{
	RX_PIN_NUM,
  TX_PIN_NUM,
	RTS_PIN_NUM,
  CTS_PIN_NUM,
  APP_UART_FLOW_CONTROL_DISABLED,
  NO_PARITY,
  NRF_UART_BAUDRATE_115200
};

// local functions
static void vHandleUartInternalErrors (uint32_t u32Error);
static void vUartErrorHandle					(app_uart_evt_t * p_event);

/**
 * @brief Public interface, initialise the FIFO UART.
 */
bool boUART_Init(void)
{
	// Initialis the nrf UART driver returning state
	uint32_t err_code;

  APP_UART_FIFO_INIT
		(	&comm_params,
			UART_RX_BUF_SIZE,
      UART_TX_BUF_SIZE,
      vUartErrorHandle,
      APP_IRQ_PRIORITY_LOWEST,
      err_code
		);

	return (err_code == NRF_SUCCESS) ? true : false;
}

bool boUART_getc(uint8_t *u8ch)
{
	bool boSuccess = false;
	
	if (app_uart_get(u8ch) == NRF_SUCCESS)
		boSuccess = true;
	
	return boSuccess;
}

static void vUartErrorHandle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
    if (p_event->evt_type == APP_UART_DATA)
    {
        // echoing symbol
        uint32_t error = app_uart_put( p_event->data.value );

        //RestartUART_timer();

        if ( !uart_rx_data_ready  ) {
            if ( p_event->data.value == '\r' ) {
                uart_rx_data_ready = true;
                app_uart_put( 0 );
            }else{
                error = app_fifo_put(&m_rx_fifo, p_event->data.value );
                if ( error == NRF_ERROR_NO_MEM ) { 
                    // buffer full, lets signal app to proceed it
                    uart_rx_data_ready = true;
                }
            }
        }
    }
}

static void vHandleUartInternalErrors (uint32_t u32Error)
{
	// notify app of error - LED ?
}


/******************************************************************************
 *
 *                              Uart Configuration
 *
 ******************************************************************************/


/* @fn  uart_error_handle
 *
 * @param[in] void
 * */

bool deca_uart_rx_data_ready()
{
    return uart_rx_data_ready;
}

void deca_uart_event_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
    if (p_event->evt_type == APP_UART_DATA)
    {
        // echoing symbol
        uint32_t error = app_uart_put( p_event->data.value );

        RestartUART_timer();

        if ( !uart_rx_data_ready  ) {
            if ( p_event->data.value == '\r' ) {
                uart_rx_data_ready = true;
                app_uart_put( 0 );
            }else{
                error = app_fifo_put(&m_rx_fifo, p_event->data.value );
                if ( error == NRF_ERROR_NO_MEM ) { 
                    // buffer full, lets signal app to proceed it
                    uart_rx_data_ready = true;
                }
            }
        }
    }
}

/* @fn  deca_uart_init
 *
 * @brief Function for initializing the UART module.
 *
 * @param[in] void
 * */
void deca_uart_init(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };
    err_code = app_fifo_init(&m_rx_fifo, rx_buf, sizeof (rx_buf));
    APP_ERROR_CHECK(err_code);

    APP_UART_INIT(&comm_params,
                    deca_uart_event_handle,
                    APP_IRQ_PRIORITY_LOW,
                    err_code);
    APP_ERROR_CHECK(err_code);

    RestartUART_timer();
}

void port_tx_msg(char *ptr, int len)
{
    for(int i=0; i<len; ++i)
    {
        while(app_uart_put(ptr[i]) != NRF_SUCCESS);
    }
}

/* @fn  deca_uart_transmit
 *
 * @brief Function for transmitting data on UART
 *
 * @param[in] ptr Pointer is contain base address of data.
 * */
void deca_uart_transmit(char *ptr)
{
    uint32_t bit=0;
    for(bit=0;ptr[bit] != '\0';bit++)
    {
        while(app_uart_put(ptr[bit]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
    while(app_uart_put('\r') != NRF_SUCCESS);
}
/* @fn  deca_uart_receive
 *
 * @brief Function for receive data from UART buffer and store into rx_buf - either full or null-terminated if partial
 *        
 * @param[in] address to buffer, max buffer size
 * @param[out] actual number of bytes in buffer
 * */
uint32_t deca_uart_receive(char * buffer, size_t size)
{
    uint32_t count = 0;
    uint32_t err_code;
    do {
        err_code = app_fifo_get( &m_rx_fifo, buffer);
        buffer++;
        count++;
    } while ( err_code == NRF_SUCCESS && count < size );
    if ( count == size ) {
        *--buffer = 0;
    }else{
        *buffer = 0;
    }
    err_code = app_uart_put( '\n' );
    app_fifo_flush( &m_rx_fifo );
    uart_rx_data_ready = false;
    return count;
}

/****************************************************************************//**
 *
 *                          End of UART Configuration
 *
 *******************************************************************************/

extern void port_tx_msg(char *ptr, int len);

#define COM_RX_BUF_SIZE   (64)

static uint8_t local_buff[COM_RX_BUF_SIZE];
static uint16_t local_buff_length=0;

typedef enum {
     NO_DATA = 0,
     COMMAND_READY,
}uart_data_e;


/*
 * @brief    Waits only commands from incoming stream.
 *             The binary interface (deca_usb2spi stream) is not allowed.
 *
 * @return  COMMAND_READY : the data for future processing can be found in app.local_buff : app.local_buff_len
 *          NO_DATA : no command yet
 */
uart_data_e waitForCommand(uint8_t *pBuf, uint16_t len)
{
    uart_data_e ret;
    static uint8_t cmdLen = 0;
    static uint8_t cmdBuf[COM_RX_BUF_SIZE]; /**< slow command buffer : small size */

    ret = NO_DATA;

    if (len <= 2)
    {/* "slow" command mode: Human interface. Wait until '\r' or '\n' */
        if (cmdLen == 0)
        {
            memset(cmdBuf, 0, sizeof(cmdBuf));
        }

        if (cmdLen < (sizeof(local_buff) - 1))
        {
            port_tx_msg(pBuf, len);    //ECHO

            if (*pBuf == '\n' || *pBuf == '\r')
            {
                if (cmdLen > 0)
                {
                    memcpy(local_buff, cmdBuf, cmdLen);

                    local_buff_length = cmdLen;
                    local_buff[cmdLen] = 0;

                    ret = COMMAND_READY;
                    cmdLen = 0;
                }
            }
            else if (*pBuf == '\b') //erase of a char in the terminal
            {
                if (cmdLen > 0)
                {
                    --cmdLen;
                    cmdBuf[cmdLen] = 0;
                    port_tx_msg((uint8_t*) "\033[K", 3);
                }

            }
            else
            {
                cmdBuf[cmdLen] = *pBuf;
                cmdLen++;
            }
        }
        else
        {
            /* error in command protocol : flush everything */
            port_tx_msg((uint8_t*) "\r\n", 2);
            cmdLen = 0;
        }
    }
    else
    {/* "fast" command mode : assume every data buffer is "COMMAND_READY" */

        if (len < (sizeof(local_buff) - 1))
        {
            memcpy(local_buff, pBuf, len);

            local_buff_length = len;
            local_buff[len] = 0;
            cmdLen = 0;

            ret = COMMAND_READY;
        }
        else
        { /* overflow in protocol : flush everything */
            port_tx_msg((uint8_t*) "Error: \r\n", 2);
            cmdLen = 0;
        }
    }

    return (ret);
}


/* @fn  process_uartmsg
 *
 * @brief Function is used for processing UARY msg.
 *        If the UART msg is dwUWB command then enter
 *        into UART_COMMAND mode and perform operation
 *        based on uart input.
 * @param[in] void
 * */
void process_uartmsg(void)
{
    char rx_buf[COM_RX_BUF_SIZE];
    int uartLen, res;

    memset(rx_buf,0,sizeof(rx_buf));
    uartLen = deca_uart_receive(rx_buf, COM_RX_BUF_SIZE );
    
    if(uartLen > 0)
    {
      res = waitForCommand(rx_buf, uartLen);
    
      if (res == COMMAND_READY)
      {
          int len = MIN((local_buff_length-1), (sizeof(local_buff)-1));
          local_buff[len+1] = 0;
          //command_parser((char *)local_buff);            //parse and execute the command
      }
    }
}

