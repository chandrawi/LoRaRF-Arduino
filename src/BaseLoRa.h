#ifndef _BASE_LORA_H_
#define _BASE_LORA_H_

#include <Arduino.h>

// #define USE_LORA_SX126X
// #define USE_LORA_SX127X

class BaseLoRa
{

#if !defined(USE_LORA_SX126X) && !defined(USE_LORA_SX127X)

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

#endif

};

#endif