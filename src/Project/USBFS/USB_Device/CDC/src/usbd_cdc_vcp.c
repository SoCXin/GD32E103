/*!
    \file  usbd_cdc_vcp.c
    \brief usb virtual ComPort media access layer functions
    
    \version 2017-12-26, V1.0.0, firmware for GD32E10x
*/

/*
    Copyright (c) 2017, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "usbd_cdc_vcp.h"
#include "usb_conf.h"

extern uint8_t app_data_buffer[];
extern line_coding_struct linecoding;
extern uint32_t app_buffer_in_ptr;

static rcu_periph_enum COM_CLK[COMn] = {EVAL_COM0_CLK, EVAL_COM1_CLK};
static uint32_t COM_TX_PIN[COMn]     = {EVAL_COM0_TX_PIN, EVAL_COM1_TX_PIN};
static uint32_t COM_RX_PIN[COMn]     = {EVAL_COM0_RX_PIN, EVAL_COM1_RX_PIN};
static uint32_t COM_GPIO_PORT[COMn]  = {EVAL_COM0_GPIO_PORT, EVAL_COM1_GPIO_PORT};
static rcu_periph_enum COM_GPIO_CLK[COMn] = {EVAL_COM0_GPIO_CLK, EVAL_COM1_GPIO_CLK};

/* Private function prototypes */
static uint16_t vcp_init          (uint8_t config);
static uint16_t vcp_deinit        (void);
static uint16_t vcp_ctrl          (uint8_t cmd, uint8_t* cmdbuf, uint16_t len);
static uint16_t vcp_data_tx       (uint32_t com, uint8_t* databuf, uint16_t len);
static uint16_t vcp_data_rx       (uint32_t com, uint8_t* databuf, uint16_t len);
static uint16_t vcp_com_config    (uint8_t config);
static void eval_com_init         (uint32_t com, uint32_t baudval, uint32_t stblen, uint32_t paritycfg, uint32_t wlen);

cdc_fop_typedef vcp_fops = 
{
    vcp_init,
    vcp_deinit,
    vcp_ctrl,
    vcp_data_tx,
    vcp_data_rx
};

/*!
    \brief      initializes the virtual com port
    \param[in]  config: configuration type
    \param[out] none
    \retval     USBD_OK if configuration type is right, USBD_FAIL else
*/
static uint16_t vcp_init(uint8_t config)
{
    if (config == DEFAULT_CONFIG) {
        /* USART interrupt configuration */
        nvic_irq_enable(USART1_IRQn, 0, 0);
        
        usart_deinit(EVAL_COM1);
        
        /* Configure and enable the USART */
        eval_com_init(EVAL_COM1, 115200, USART_STB_1BIT, USART_PM_NONE, USART_WL_8BIT);

        /* Enable USART Receive Interrupt */
        usart_interrupt_enable(EVAL_COM1, USART_INT_RBNE);

        return USBD_OK;
    } else {
        return USBD_FAIL;
    }
}

/*!
    \brief      Deinitializes the virtual com port
    \param[in]  none
    \param[out] none
    \retval     USBD_OK
*/
static uint16_t vcp_deinit(void)
{
    return USBD_OK;
}

/*!
    \brief      Handle the CDC device class requests
    \param[in]  cmd: command code
    \param[in]  cmdbuf: command data buffer pointer
    \param[in]  len: count of data to be sent (in bytes)
    \param[out] none
    \retval     USBD_OK
*/
static uint16_t vcp_ctrl (uint8_t cmd, uint8_t* cmdbuf, uint16_t len)
{
    switch (cmd) {
    case SEND_ENCAPSULATED_COMMAND:
        /* No operation for this driver */
        break;

    case GET_ENCAPSULATED_RESPONSE:
        /* No operation for this driver */
        break;

    case SET_COMM_FEATURE:
        /* No operation for this driver */
        break;

    case GET_COMM_FEATURE:
        /* No operation for this driver */
        break;

    case CLEAR_COMM_FEATURE:
        /* No operation for this driver */
        break;

    case SET_LINE_CODING:
        linecoding.dwDTERate = (uint32_t)(cmdbuf[0] | (cmdbuf[1] << 8) | (cmdbuf[2] << 16) | (cmdbuf[3] << 24));
        linecoding.bCharFormat = cmdbuf[4];
        linecoding.bParityType = cmdbuf[5];
        linecoding.bDataBits = cmdbuf[6];
        vcp_com_config(SETTING_CONFIG);  /* Set the new configuration */
        break;

    case GET_LINE_CODING:
        cmdbuf[0] = (uint8_t)(linecoding.dwDTERate);
        cmdbuf[1] = (uint8_t)(linecoding.dwDTERate >> 8);
        cmdbuf[2] = (uint8_t)(linecoding.dwDTERate >> 16);
        cmdbuf[3] = (uint8_t)(linecoding.dwDTERate >> 24);
        cmdbuf[4] = linecoding.bCharFormat;
        cmdbuf[5] = linecoding.bParityType;
        cmdbuf[6] = linecoding.bDataBits;
        break;

    case SET_CONTROL_LINE_STATE:
        /* No operation for this driver */
        break;

    case SEND_BREAK:
        /* No operation for this driver */
        break;

    default:
        break;
    }

    return USBD_OK;
}

/*!
    \brief      Real usart received data to be send over virtual usart(USB Tx endpoint)
    \param[in]  com: specifies the COM port to be configured
    \param[in]  databuf: data buffer will be sent
    \param[in]  len: count of data to be sent (in bytes)
    \param[out] none
    \retval     USBD_OK if all operation is right, else USBD_FAIL
*/
static uint16_t vcp_data_tx (uint32_t com, uint8_t* databuf, uint16_t len)
{
    if (linecoding.bDataBits == 7) {
    
        app_data_buffer[app_buffer_in_ptr] = usart_data_receive(com) & 0x7F;
    } else if (linecoding.bDataBits == 8) {
    
        app_data_buffer[app_buffer_in_ptr] = usart_data_receive(com);
    }

    app_buffer_in_ptr++;

    /* To avoid buffer overflow */
    if (app_buffer_in_ptr == APP_RX_DATA_SIZE) {
        app_buffer_in_ptr = 0;
    }

    return USBD_OK;
}

/*!
    \brief      Virtual usart received over USB Rx endpoint are sent over real usart
    \param[in]  com: specifies the COM port to be configured
    \param[in]  databuf: data buffer will be sent
    \param[in]  len: count of data to be sent (in bytes)
    \param[out] none
    \retval     USBD_OK
*/
static uint16_t vcp_data_rx (uint32_t com, uint8_t* databuf, uint16_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++) {
        usart_data_transmit(com, *(databuf + i) );
        while(usart_flag_get(com, USART_FLAG_TBE) == RESET); 
    }

    return USBD_OK;
}

/*!
    \brief      Configure the ComPort with default values or values received from host
    \param[in]  config: can be DEFAULT_CONFIG to have no operation or SETTING_CONFIG 
                        to set a configuration received from the host
    \param[out] none
    \retval     USBD_FAIL if received command data have errors, or will be USBD_OK
*/
static uint16_t vcp_com_config(uint8_t config)
{
    uint32_t usart_stop_bits;
    uint32_t usart_parity;
    uint32_t usart_word_len;
    uint32_t usart_baud_rate;

    if (config == SETTING_CONFIG) {
        /* set the Stop bit*/
        switch (linecoding.bCharFormat) {
        case 0:
            usart_stop_bits = USART_STB_1BIT;
            break;

        case 1:
            usart_stop_bits = USART_STB_1_5BIT;
            break;

        case 2:
            usart_stop_bits = USART_STB_2BIT;
            break;

        default :
            return (USBD_FAIL);
        }

        /* set the parity bit*/
        switch (linecoding.bParityType) {
        case 0:
            usart_parity = USART_PM_NONE;
            break;

        case 1:
            usart_parity = USART_PM_EVEN;
            break;

        case 2:
            usart_parity = USART_PM_ODD;
            break;

        default :
            return (USBD_FAIL);
        }

        /*set the data type : only 8bits and 9bits is supported */
        switch (linecoding.bDataBits) {
        case 0x07:
            /* With this configuration a parity (Even or Odd) should be set */
            usart_word_len = USART_WL_8BIT;
            break;

        case 0x08:
            if (usart_parity == USART_PM_NONE) {
                usart_word_len = USART_WL_8BIT;
            } else {
                usart_word_len = USART_WL_9BIT;
            }
            break;

        default :
            return (USBD_FAIL);
        }

        usart_baud_rate = linecoding.dwDTERate;

        /* Configure and enable the USART */
        eval_com_init(EVAL_COM1, usart_baud_rate, usart_stop_bits, usart_parity, usart_word_len);
    }

    return USBD_OK;
}

/*!
    \brief      USART1 intterupt handler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EVAL_COM2_IRQHandler (void)
{
    if (usart_interrupt_flag_get(EVAL_COM1, USART_INT_FLAG_RBNE) != RESET) {
        /* Send the received data to the PC Host*/
        vcp_data_tx (EVAL_COM1, 0, 0);
    }

    /* If overrun condition occurs, clear the ORE flag and recover communication */
    if (usart_interrupt_flag_get(EVAL_COM1, USART_INT_FLAG_ERR_ORERR) != RESET) {
        usart_data_receive(EVAL_COM1);
    }
}

/*!
    \brief      configures com port
    \param[in]  com: specifies the COM port to be configured
    \param[in]  baudval: baud rate value
    \param[in]  stblen: USART stop bit configure
    \param[in]  paritycfg: configure USART parity
    \param[in]  wlen: USART word length configure
    \param[out] none
    \retval     none
*/
static void eval_com_init(uint32_t com, uint32_t baudval, uint32_t stblen, uint32_t paritycfg, uint32_t wlen)
{
    uint32_t com_id = 0U;

    if (EVAL_COM0 == com) {
        com_id = 0U;
    } else if (EVAL_COM1 == com) {
        com_id = 1U;
    }

    /* enable GPIO clock */
    rcu_periph_clock_enable(COM_GPIO_CLK[com_id]);

    /* enable USART clock */
    rcu_periph_clock_enable(COM_CLK[com_id]);

    /* connect port to USARTx_Tx */
    gpio_init(COM_GPIO_PORT[com_id], GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, COM_TX_PIN[com_id]);

    /* connect port to USARTx_Rx */
    gpio_init(COM_GPIO_PORT[com_id], GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, COM_RX_PIN[com_id]);

    /* USART configure */
    usart_baudrate_set(com, baudval);
    usart_parity_config(com, paritycfg);
    usart_word_length_set(com, wlen);
    usart_stop_bit_set(com, stblen);
    usart_receive_config(com, USART_RECEIVE_ENABLE);
    usart_transmit_config(com, USART_TRANSMIT_ENABLE);
    usart_enable(com);
}
