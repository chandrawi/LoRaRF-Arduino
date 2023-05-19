#ifndef _SX126X_H_
#define _SX126X_H_

#include <BaseLoRa.h>
#include <SX126x_driver.h>

// Status TX and RX operation
#define SX126X_STATUS_DEFAULT                   LORA_STATUS_DEFAULT
#define SX126X_STATUS_TX_WAIT                   LORA_STATUS_TX_WAIT
#define SX126X_STATUS_TX_TIMEOUT                LORA_STATUS_TX_TIMEOUT
#define SX126X_STATUS_TX_DONE                   LORA_STATUS_TX_DONE
#define SX126X_STATUS_RX_WAIT                   LORA_STATUS_RX_WAIT
#define SX126X_STATUS_RX_CONTINUOUS             LORA_STATUS_RX_CONTINUOUS
#define SX126X_STATUS_RX_TIMEOUT                LORA_STATUS_RX_TIMEOUT
#define SX126X_STATUS_RX_DONE                   LORA_STATUS_RX_DONE
#define SX126X_STATUS_HEADER_ERR                LORA_STATUS_HEADER_ERR
#define SX126X_STATUS_CRC_ERR                   LORA_STATUS_CRC_ERR
#define SX126X_STATUS_CAD_WAIT                  LORA_STATUS_CAD_WAIT
#define SX126X_STATUS_CAD_DETECTED              LORA_STATUS_CAD_DETECTED
#define SX126X_STATUS_CAD_DONE                  LORA_STATUS_CAD_DONE

// Default Hardware Configuration
#define SX126X_PIN_RF_IRQ                             1

#if defined(USE_LORA_SX126X) && defined(USE_LORA_SX127X)
class SX126x : public BaseLoRa
#else
class SX126x
#endif
{

    public:

        SX126x();

        // Common Operational methods
        bool begin();
        bool begin(int8_t nss, int8_t reset, int8_t busy, int8_t irq=-1, int8_t txen=-1, int8_t rxen=-1);
        void end();
        bool reset();
        void sleep(uint8_t option=SX126X_SLEEP_WARM_START);
        void wake();
        void standby(uint8_t option=SX126X_STANDBY_RC);
        void setActive();
        bool busyCheck(uint32_t timeout=SX126X_BUSY_TIMEOUT);
        void setFallbackMode(uint8_t fallbackMode);
        uint8_t getMode();

        // Hardware configuration methods
        void setSPI(SPIClass &SpiObject, uint32_t frequency=SX126X_SPI_FREQUENCY);
        void setPins(int8_t nss, int8_t reset, int8_t busy, int8_t irq=-1, int8_t txen=-1, int8_t rxen=-1);
        void setRfIrqPin(int8_t dioPinSelect);
        void setDio2RfSwitch(bool enable=true);
        void setDio3TcxoCtrl(uint8_t tcxoVoltage, uint32_t delayTime);
        void setXtalCap(uint8_t xtalA, uint8_t xtalB);
        void setRegulator(uint8_t regMode);
        void setCurrentProtection(uint8_t current);

        // Modem, modulation parameter, and packet parameter setup methods
        uint8_t getModem();
        void setModem(uint8_t modem=SX126X_LORA_MODEM);
        void setFrequency(uint32_t frequency);
        void setTxPower(uint8_t txPower, uint8_t version=SX126X_TX_POWER_SX1262);
        void setRxGain(uint8_t boost);
        void setLoRaModulation(uint8_t sf, uint32_t bw, uint8_t cr, bool ldro=false);
        void setLoRaPacket(uint8_t headerType, uint16_t preambleLength, uint8_t payloadLength, bool crcType=false, bool invertIq=false);
        void setSpreadingFactor(uint8_t sf);
        void setBandwidth(uint32_t bw);
        void setCodeRate(uint8_t cr);
        void setLdroEnable(bool ldro=true);
        void setHeaderType(uint8_t headerType);
        void setPreambleLength(uint16_t preambleLength);
        void setPayloadLength(uint8_t payloadLength);
        void setCrcEnable(bool crcType=true);
        void setInvertIq(bool invertIq=true);
        void setSyncWord(uint16_t syncWord);
        void setFskModulation(uint32_t br, uint8_t pulseShape, uint8_t bandwidth, uint32_t Fdev);
        void setFskPacket(uint16_t preambleLength, uint8_t preambleDetector, uint8_t syncWordLength, uint8_t addrComp, uint8_t packetType, uint8_t payloadLength, uint8_t crcType, uint8_t whitening);
        void setFskSyncWord(uint8_t* sw, uint8_t swLen);
        void setFskAdress(uint8_t nodeAddr, uint8_t broadcastAddr);
        void setFskCrc(uint16_t crcInit, uint16_t crcPolynom);
        void setFskWhitening(uint16_t whitening);

        // Transmit related methods
        void beginPacket();
        bool endPacket(uint32_t timeout=SX126X_TX_SINGLE);
        void write(uint8_t data);
        void write(uint8_t* data, uint8_t length);
        void write(char* data, uint8_t length);
        template <typename T> void put(T data)
        {
            const uint8_t length = sizeof(T);
            union conv {
                T Data;
                uint8_t Binary[length];
            };
            union conv u;
            u.Data = data;
            sx126x_writeBuffer(_bufferIndex, u.Binary, length);
            _bufferIndex += length;
            _payloadTxRx += length;
        }
        void onTransmit(void(&callback)());

        // Receive related methods
        bool request(uint32_t timeout=SX126X_RX_SINGLE);
        bool listen(uint32_t rxPeriod, uint32_t sleepPeriod);
        uint8_t available();
        uint8_t read();
        uint8_t read(uint8_t* data, uint8_t length);
        uint8_t read(char* data, uint8_t length);
        void purge(uint8_t length=0);
        template <typename T> uint8_t get(T &data)
        {
            const uint8_t length = sizeof(T);
            union conv {
                T Data;
                uint8_t Binary[length];
            };
            union conv u;
            sx126x_readBuffer(_bufferIndex, u.Binary, length);
            data = u.Data;
            _bufferIndex += length;
            _payloadTxRx = _payloadTxRx > length ? _payloadTxRx - length : 0;
            return _payloadTxRx > length ? length : _payloadTxRx;
        }
        void onReceive(void(&callback)());

        // Wait, operation status, and packet status methods
        bool wait(uint32_t timeout=0);
        uint8_t status();
        uint32_t transmitTime();
        float dataRate();
        int16_t packetRssi();
        float snr();
        int16_t signalRssi();
        int16_t rssiInst();
        uint16_t getError();
        uint32_t random();

    protected:

        uint8_t _modem;
        uint8_t _sf = 7;
        uint32_t _bw = 125000;
        uint8_t _cr = 4;
        bool _ldro;
        uint8_t _headerType = SX126X_HEADER_EXPLICIT;
        uint16_t _preambleLength = 12;
        uint8_t _payloadLength;
        bool _crcType;
        bool _invertIq;
        static void (*_onTransmit)();
        static void (*_onReceive)();

    private:

        SPIClass* _spi;
        int8_t _nss, _reset, _busy, _irq, _txen, _rxen;
        int8_t _dio;
        uint8_t _statusWait;
        volatile static uint16_t _statusIrq;
        static uint32_t _transmitTime;
        static uint8_t _bufferIndex;
        static uint8_t _payloadTxRx;
        static int8_t _irqStatic;
        static int8_t _pinToLow;
        uint16_t _random;

        // Interrupt handler methods
        void _irqSetup(uint16_t irqMask);
#ifdef ESP8266
        static void ICACHE_RAM_ATTR _interruptTx();
        static void ICACHE_RAM_ATTR _interruptRx();
        static void ICACHE_RAM_ATTR _interruptRxContinuous();
#else
        static void _interruptTx();
        static void _interruptRx();
        static void _interruptRxContinuous();
#endif

};

#endif