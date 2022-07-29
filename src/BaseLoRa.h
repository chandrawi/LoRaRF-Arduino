#ifndef _BASE_LORA_H_
#define _BASE_LORA_H_

#include <Arduino.h>

// Modem options
#define FSK_MODEM                               0x00        // GFSK packet type
#define LORA_MODEM                              0x01        // LoRa packet type

// RX gain options
#define LORA_RX_GAIN_POWER_SAVING               0x00        // gain used in Rx mode: power saving gain
#define LORA_RX_GAIN_BOOSTED                    0x01        //                       boosted gain

// Header type
#define LORA_HEADER_EXPLICIT                    0x00        // explicit header mode
#define LORA_HEADER_IMPLICIT                    0x01        // implicit header mode

// TX and RX operation mode
#define LORA_TX_SINGLE                          0x000000    // Tx timeout duration: no timeout (Rx single mode)
#define LORA_RX_SINGLE                          0x000000    // Rx timeout duration: no timeout (Rx single mode)
#define LORA_RX_CONTINUOUS                      0xFFFFFF    //                      infinite (Rx continuous mode)

// Status TX and RX operation
#define LORA_STATUS_DEFAULT                     0           // default status (false)
#define LORA_STATUS_TX_WAIT                     1
#define LORA_STATUS_TX_TIMEOUT                  2
#define LORA_STATUS_TX_DONE                     3
#define LORA_STATUS_RX_WAIT                     4
#define LORA_STATUS_RX_CONTINUOUS               5
#define LORA_STATUS_RX_TIMEOUT                  6
#define LORA_STATUS_RX_DONE                     7
#define LORA_STATUS_HEADER_ERR                  8
#define LORA_STATUS_CRC_ERR                     9
#define LORA_STATUS_CAD_WAIT                    10
#define LORA_STATUS_CAD_DETECTED                11
#define LORA_STATUS_CAD_DONE                    12

// Uncomment one of line below to use one or more LoRa model for a network library
#define USE_LORA_SX126X
#define USE_LORA_SX127X

class BaseLoRa
{

    public:

        virtual void beginPacket();
        virtual bool endPacket(uint32_t timeout);
        virtual void write(uint8_t data);
        virtual void write(uint8_t* data, uint8_t length);

        virtual bool request(uint32_t timeout);
        virtual uint8_t available();
        virtual uint8_t read();
        virtual uint8_t read(uint8_t* data, uint8_t length);

        virtual bool wait(uint32_t timeout);
        virtual uint8_t status();

};

#endif