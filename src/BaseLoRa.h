#ifndef _BASE_LORA_H_
#define _BASE_LORA_H_

#include <Arduino.h>

// Status TX and RX operation
#define LORA_STATUS_DEFAULT                         0           // default status (false)
#define LORA_STATUS_TX_WAIT                         1
#define LORA_STATUS_TX_TIMEOUT                      2
#define LORA_STATUS_TX_DONE                         3
#define LORA_STATUS_RX_WAIT                         4
#define LORA_STATUS_RX_CONTINUOUS_WAIT              5
#define LORA_STATUS_RX_TIMEOUT                      6
#define LORA_STATUS_RX_DONE                         7
#define LORA_STATUS_HEADER_ERR                      8
#define LORA_STATUS_CRC_ERR                         9
#define LORA_STATUS_CAD_WAIT                        10
#define LORA_STATUS_CAD_DETECTED                    11
#define LORA_STATUS_CAD_DONE                        12

// Uncomment one of line below to use a LoRa model exclusively
#define USE_LORA_SX126X
// #define USE_LORA_SX127X

class BaseLoRa
{

    public:

        virtual void beginPacket();
        virtual void endPacket(uint32_t timeout);
        virtual void write(uint8_t data);
        virtual void write(uint8_t* data, uint8_t length);
        virtual void write(char* data, uint8_t length);

        virtual void request(uint32_t timeout);
        virtual void listen(uint32_t rxPeriod, uint32_t sleepPeriod);
        virtual uint8_t available();
        virtual uint8_t read();
        virtual uint8_t read(uint8_t* data, uint8_t length);
        virtual uint8_t read(char* data, uint8_t length);
        virtual void setLoRaPayloadLength(uint8_t length);

        virtual uint8_t status();
        virtual void wait();

};

#endif