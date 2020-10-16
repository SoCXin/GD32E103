/*!
    \file  cdc_acm_core.c
    \brief CDC ACM driver
    
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

#include "usbd_int.h"
#include "cdc_acm_core.h"
#include "usbd_cdc_vcp.h"

#define USBD_VID                          0x28E9
#define USBD_PID                          0x018A

static void cdc_acm_data_receive(void *pudev);
static void cdc_acm_data_send (void *pudev);
static uint8_t cdc_acm_sof (usb_core_handle_struct *pudev);
static void cdc_acm_asynch_xfer_handle (usb_core_handle_struct *pudev);

static uint32_t cdc_cmd = 0xFF;
static uint32_t cdc_len = 0;
static __IO uint32_t usbd_cdc_altset = 0;

__ALIGN_BEGIN uint8_t usb_data_buffer[CDC_ACM_DATA_PACKET_SIZE] __ALIGN_END;
__ALIGN_BEGIN uint8_t usb_cmd_buffer[CDC_ACM_CMD_PACKET_SIZE] __ALIGN_END;

usbd_int_cb_struct cdc_acm_fops = {cdc_acm_sof};
usbd_int_cb_struct *usbd_int_fops = &cdc_acm_fops;


uint8_t app_data_buffer[APP_RX_DATA_SIZE];
__IO uint32_t receive_length = 0;
__IO uint8_t usb_tx_state = 0;
__IO uint32_t end_packet = 0;

uint32_t app_buffer_in_ptr  = 0;
uint32_t app_buffer_out_ptr = 0;
uint32_t app_rx_length  = 0;

__ALIGN_BEGIN line_coding_struct linecoding __ALIGN_END =
{
    115200, /* baud rate     */
    0x00,   /* stop bits - 1 */
    0x00,   /* parity - none */
    0x08    /* num of bits 8 */
};

/* note:it should use the C99 standard when compiling the below codes */
/* USB standard device descriptor */
__ALIGN_BEGIN const usb_descriptor_device_struct device_descriptor __ALIGN_END =
{
    .Header = 
     {
         .bLength = USB_DEVICE_DESC_SIZE, 
         .bDescriptorType = USB_DESCTYPE_DEVICE
     },
    .bcdUSB = 0x0200,
    .bDeviceClass = 0x02,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = USB_MAX_EP0_SIZE,
    .idVendor = USBD_VID,
    .idProduct = USBD_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = USBD_MFC_STR_IDX,
    .iProduct = USBD_PRODUCT_STR_IDX,
    .iSerialNumber = USBD_SERIAL_STR_IDX,
    .bNumberConfigurations = USBD_CFG_MAX_NUM
};

/* USB device configuration descriptor */
__ALIGN_BEGIN const usb_descriptor_configuration_set_struct configuration_descriptor __ALIGN_END = 
{
    .config = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_configuration_struct), 
            .bDescriptorType = USB_DESCTYPE_CONFIGURATION
         },
        .wTotalLength = USB_CDC_ACM_CONFIG_DESC_SIZE,
        .bNumInterfaces = 0x02,
        .bConfigurationValue = 0x01,
        .iConfiguration = 0x00,
        .bmAttributes = 0x80,
        .bMaxPower = 0x32
    },

    .cdc_loopback_interface = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_interface_struct), 
             .bDescriptorType = USB_DESCTYPE_INTERFACE 
         },
        .bInterfaceNumber = 0x00,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x01,
        .bInterfaceClass = 0x02,
        .bInterfaceSubClass = 0x02,
        .bInterfaceProtocol = 0x01,
        .iInterface = 0x00
    },

    .cdc_loopback_header = 
    {
        .Header =
         {
            .bLength = sizeof(usb_descriptor_header_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x00,
        .bcdCDC = 0x0110
    },

    .cdc_loopback_call_managment = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_call_managment_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x01,
        .bmCapabilities = 0x00,
        .bDataInterface = 0x01
    },

    .cdc_loopback_acm = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_acm_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x02,
        .bmCapabilities = 0x02,
    },

    .cdc_loopback_union = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_union_function_struct), 
            .bDescriptorType = USB_DESCTYPE_CS_INTERFACE
         },
        .bDescriptorSubtype = 0x06,
        .bMasterInterface = 0x00,
        .bSlaveInterface0 = 0x01,
    },

    .cdc_loopback_cmd_endpoint = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_endpoint_struct), 
            .bDescriptorType = USB_DESCTYPE_ENDPOINT
         },
        .bEndpointAddress = CDC_ACM_CMD_EP,
        .bmAttributes = 0x03,
        .wMaxPacketSize = CDC_ACM_CMD_PACKET_SIZE,
        .bInterval = 0x0A
    },

    .cdc_loopback_data_interface = 
    {
        .Header = 
         {
            .bLength = sizeof(usb_descriptor_interface_struct), 
            .bDescriptorType = USB_DESCTYPE_INTERFACE
         },
        .bInterfaceNumber = 0x01,
        .bAlternateSetting = 0x00,
        .bNumEndpoints = 0x02,
        .bInterfaceClass = 0x0A,
        .bInterfaceSubClass = 0x00,
        .bInterfaceProtocol = 0x00,
        .iInterface = 0x00
    },

    .cdc_loopback_out_endpoint = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_endpoint_struct), 
             .bDescriptorType = USB_DESCTYPE_ENDPOINT 
         },
        .bEndpointAddress = CDC_ACM_DATA_OUT_EP,
        .bmAttributes = 0x02,
        .wMaxPacketSize = CDC_ACM_DATA_PACKET_SIZE,
        .bInterval = 0x00
    },

    .cdc_loopback_in_endpoint = 
    {
        .Header = 
         {
             .bLength = sizeof(usb_descriptor_endpoint_struct), 
             .bDescriptorType = USB_DESCTYPE_ENDPOINT 
         },
        .bEndpointAddress = CDC_ACM_DATA_IN_EP,
        .bmAttributes = 0x02,
        .wMaxPacketSize = CDC_ACM_DATA_PACKET_SIZE,
        .bInterval = 0x00
    }
};

/* USB language ID Descriptor */
__ALIGN_BEGIN const usb_descriptor_language_id_struct usbd_language_id_desc __ALIGN_END = 
{
    .Header = 
     {
         .bLength = sizeof(usb_descriptor_language_id_struct), 
         .bDescriptorType = USB_DESCTYPE_STRING
     },
    .wLANGID = ENG_LANGID
};

__ALIGN_BEGIN uint8_t* usbd_strings[] __ALIGN_END = 
{
    [USBD_LANGID_STR_IDX] = (uint8_t *)&usbd_language_id_desc,
    [USBD_MFC_STR_IDX] = USBD_STRING_DESC("GigaDevice"),
    [USBD_PRODUCT_STR_IDX] = USBD_STRING_DESC("GD32 USB CDC ACM"),
    [USBD_SERIAL_STR_IDX] = USBD_STRING_DESC("GD32E10X-1.0.0-7z8x9yer")
};

/*!
    \brief      initialize the CDC ACM device
    \param[in]  pudev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
uint8_t cdc_acm_init (void *pudev, uint8_t config_index)
{
    /* initialize the data Tx/Rx endpoint */
    usbd_ep_init(pudev, &(configuration_descriptor.cdc_loopback_in_endpoint));
    usbd_ep_init(pudev, &(configuration_descriptor.cdc_loopback_out_endpoint));

    /* initialize the command Tx endpoint */
    usbd_ep_init(pudev, &(configuration_descriptor.cdc_loopback_cmd_endpoint));
    
    /* Initialize the Interface physical components */
    vcp_fops.pIf_Init(DEFAULT_CONFIG);
    
    /* Enable OUT endpoint prepare to receive next packet */
    usbd_ep_rx(pudev, CDC_ACM_DATA_OUT_EP, (uint8_t*)(usb_data_buffer), CDC_ACM_DATA_PACKET_SIZE);

    return USBD_OK;
}

/*!
    \brief      de-initialize the CDC ACM device
    \param[in]  pudev: pointer to USB device instance
    \param[in]  config_index: configuration index
    \param[out] none
    \retval     USB device operation status
*/
uint8_t cdc_acm_deinit (void *pudev, uint8_t config_index)
{
    /* deinitialize the data Tx/Rx endpoint */
    usbd_ep_deinit(pudev, CDC_ACM_DATA_IN_EP);
    usbd_ep_deinit(pudev, CDC_ACM_DATA_OUT_EP);

    /* deinitialize the command Tx endpoint */
    usbd_ep_deinit(pudev, CDC_ACM_CMD_EP);
    
    /* Restore default state of the interface physical components */
    vcp_fops.pIf_DeInit();

    return USBD_OK;
}

/*!
    \brief      handle CDC ACM data
    \param[in]  pudev: pointer to USB device instance
    \param[in]  rx_tx: data transfer direction:
      \arg        USBD_TX
      \arg        USBD_RX
    \param[in]  ep_id: endpoint identifier
    \param[out] none
    \retval     USB device operation status
*/
uint8_t cdc_acm_data_handler (void *pudev, usb_dir_enum rx_tx, uint8_t ep_num)
{
    if ((USB_TX == rx_tx) && ((CDC_ACM_DATA_IN_EP & 0x7F) == ep_num)) {
        usb_ep_struct *ep = &((usb_core_handle_struct*)pudev)->dev.in_ep[ep_num];
        
        if ((ep->xfer_len % ep->endp_mps == 0) && (ep->xfer_len != 0)) {
            usbd_ep_tx (pudev, ep_num, NULL, 0U);
        } else {
            cdc_acm_data_send (pudev);
        }
        return USBD_OK;
    } else if ((USB_RX == rx_tx) && ((EP0_OUT & 0x7F) == ep_num)) {
        cdc_acm_EP0_RxReady (pudev);
    } else if ((USB_RX == rx_tx) && ((CDC_ACM_DATA_OUT_EP & 0x7F) == ep_num)) {
        receive_length = usbd_rxcount_get(pudev, CDC_ACM_DATA_OUT_EP);
        cdc_acm_data_receive (pudev);
        return USBD_OK;
    }

    return USBD_FAIL;
}

/*!
    \brief      handle the CDC ACM class-specific requests
    \param[in]  pudev: pointer to USB device instance
    \param[in]  req: device class-specific request
    \param[out] none
    \retval     USB device operation status
*/
uint8_t cdc_acm_req_handler (void *pudev, usb_device_req_struct *req)
{
    uint16_t len = CDC_ACM_DESC_SIZE;
    uint8_t  *pbuf= (uint8_t*)(&configuration_descriptor) + 9;

    switch (req->bmRequestType & USB_REQ_MASK) {
    case USB_CLASS_REQ:
        if (req->bmRequestType & 0x80) {
            /* Get the request data from media access layer */
            vcp_fops.pIf_Ctrl(req->bRequest, usb_cmd_buffer, req->wLength);
            /* send the request data to the host */
            usbd_ctltx (pudev, usb_cmd_buffer, req->wLength);
        /* Request is from host to device */        
        } else {
            /* Set the value of the current command to be processed */
            cdc_cmd = req->bRequest;
            cdc_len = req->wLength;

            /* Enable EP0 prepare to receive command data packet */
            usbd_ctlrx (pudev, usb_cmd_buffer, req->wLength);
        }
        break;
    case USB_STANDARD_REQ:
        /* standard device request */
        switch(req->bRequest) {
        case USBREQ_GET_INTERFACE:
            usbd_ctltx(pudev, (uint8_t *)&usbd_cdc_altset, 1);
            break;
        case USBREQ_SET_INTERFACE:
            if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM) {
                usbd_cdc_altset = (uint8_t)(req->wValue);
            } else {
                /* call the error management function (command will be nacked) */
                usbd_enum_error (pudev, req);
            }
            break;
        case USBREQ_GET_DESCRIPTOR:
            if(CDC_ACM_DESC_TYPE == (req->wValue >> 8)){
                len = USB_MIN(CDC_ACM_DESC_SIZE, req->wLength);
                pbuf = (uint8_t*)(&configuration_descriptor) + 9 + (9 * USBD_ITF_MAX_NUM);
            }

            usbd_ctltx(pudev, pbuf, len);
            break;
        default:
            break;
        }
    default:
        break;
    }

    return USBD_OK;
}

/*!
    \brief      receive CDC ACM data
    \param[in]  pudev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
static void cdc_acm_data_receive(void *pudev)
{
    /* USB received data will be immediately processed in media access layer */
    vcp_fops.pIf_DataRx(EVAL_COM1, usb_data_buffer, receive_length);

    /* Enable Out endpoint prepare to receive next data packet */
    usbd_ep_rx(pudev, CDC_ACM_DATA_OUT_EP, (uint8_t*)(usb_data_buffer), CDC_ACM_DATA_PACKET_SIZE);
}

/*!
    \brief      send CDC ACM data
    \param[in]  pudev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
static void cdc_acm_data_send (void *pudev)
{
    uint16_t usb_tx_length;

    if (usb_tx_state == 1) {
        if (app_rx_length == 0) {
            if (end_packet == 1) {
                end_packet = 0;

                /*Send zero-length packet*/
                usbd_ep_tx(pudev, CDC_ACM_DATA_IN_EP, 0, 0);
            } else {
                usb_tx_state = 0;
            }
        } else {
            if (app_rx_length > CDC_ACM_DATA_PACKET_SIZE) {
                usb_tx_length = CDC_ACM_DATA_PACKET_SIZE;
            } else {
                usb_tx_length = app_rx_length;
                if (app_rx_length == CDC_ACM_DATA_PACKET_SIZE) end_packet = 1;
            }

            /* Prepare the available data buffer to be sent on IN endpoint */
            usbd_ep_tx(pudev,
                       CDC_ACM_DATA_IN_EP,
                       &app_data_buffer[app_buffer_out_ptr],
                       usb_tx_length);

            app_buffer_out_ptr += usb_tx_length;
            app_rx_length  -= usb_tx_length;
        }
    }
}

/*!
    \brief      command data received on control endpoint
    \param[in]  pudev: pointer to USB device instance
    \param[out] none
    \retval     USB device operation status
*/
usbd_status_enum cdc_acm_EP0_RxReady (void *pudev)
{
    if (NO_CMD != cdc_cmd) {
        /* Process the command data */
        vcp_fops.pIf_Ctrl(cdc_cmd, usb_cmd_buffer, cdc_len);

        cdc_cmd = NO_CMD;
        cdc_len = 0;
    }

    return USBD_OK;
}

/*!
    \brief      handle start of frame event
    \param[in]  pudev: pointer to usb device instance
    \param[out] none
    \retval     USB device operation status
*/
static uint8_t cdc_acm_sof (usb_core_handle_struct *pudev)
{
    static uint32_t FrameCount = 0;

    if (FrameCount++ == CDC_IN_FRAME_INTERVAL) {
        /* Frame counter is reset */
        FrameCount = 0;

        /* Handle asynchronous transmission between USB endpoint and Usart */
        cdc_acm_asynch_xfer_handle(pudev);
    }

    return USBD_OK;
}

/*!
    \brief      prepare send data over CDC IN endpoint
    \param[in]  pudev: pointer to usb device instance
    \param[out] none
    \retval     none
*/
static void cdc_acm_asynch_xfer_handle (usb_core_handle_struct *pudev)
{
    uint16_t usb_tx_length;

    if (usb_tx_state != 1) {
        if (app_buffer_out_ptr == app_buffer_in_ptr) {
            /* No data received by real usart */
            return;
        }

        if(app_buffer_out_ptr > app_buffer_in_ptr) { 
            /* Need rollback */
            app_rx_length = APP_RX_DATA_SIZE - app_buffer_out_ptr;
        } else {
            app_rx_length = app_buffer_in_ptr - app_buffer_out_ptr;
        }

        if (app_rx_length > CDC_ACM_DATA_PACKET_SIZE) {
            usb_tx_length = CDC_ACM_DATA_PACKET_SIZE;
        } else {
            usb_tx_length = app_rx_length;
            if (usb_tx_length == CDC_ACM_DATA_PACKET_SIZE) {
                end_packet = 1; /* Last packet will be sent */
            }
        }

        usbd_ep_tx(pudev,
                   CDC_ACM_DATA_IN_EP,
                   &app_data_buffer[app_buffer_out_ptr],
                   usb_tx_length);

        /* Update Tx parameters */
        app_buffer_out_ptr += usb_tx_length;
        app_rx_length  -= usb_tx_length;

        usb_tx_state = 1;

        if (app_buffer_out_ptr == APP_RX_DATA_SIZE) {
            app_buffer_out_ptr = 0;
        }
    }
}
