#ifndef ELMulator_CANBus_h
#define ELMulator_CANBus_h

#include <Arduino.h>
#include "driver/twai.h"

// WaveShare ESP32-S3-RS485-CAN default pins
#ifndef CAN_TX_PIN
#define CAN_TX_PIN GPIO_NUM_15
#endif

#ifndef CAN_RX_PIN
#define CAN_RX_PIN GPIO_NUM_16
#endif

// ISO-TP constants
#define ISOTP_SINGLE_FRAME     0x00
#define ISOTP_FIRST_FRAME      0x10
#define ISOTP_CONSECUTIVE_FRAME 0x20
#define ISOTP_FLOW_CONTROL     0x30

#define ISOTP_MAX_DATA_SIZE    4095
#define CAN_RESPONSE_TIMEOUT_MS 2000
#define ISOTP_FC_WAIT_MS       50
#define ISOTP_CF_INTERVAL_MS   10

class CANBus
{
public:
    CANBus(gpio_num_t txPin = CAN_TX_PIN, gpio_num_t rxPin = CAN_RX_PIN);
    ~CANBus();

    bool begin(uint32_t baudRate = 500000);
    void stop();

    // Send an ISO-TP request and receive the response
    // txId: CAN arbitration ID for the request (e.g. 0x7DF for broadcast, 0x7E0 for ECU)
    // rxId: CAN arbitration ID to listen for response (e.g. 0x7E8)
    // data: raw diagnostic bytes to send
    // dataLen: length of data
    // response: buffer for response data
    // responseLen: on return, length of response data
    // maxResponseLen: size of response buffer
    // Returns true on success
    bool sendISOTP(uint32_t txId, uint32_t rxId,
                   const uint8_t *data, uint16_t dataLen,
                   uint8_t *response, uint16_t *responseLen,
                   uint16_t maxResponseLen);

    // Send raw CAN frame
    bool sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool extendedId = false);

    // Receive raw CAN frame with timeout
    bool receiveFrame(twai_message_t *message, uint32_t timeoutMs = CAN_RESPONSE_TIMEOUT_MS);

    // Monitor mode - receive any CAN frame
    bool receiveAny(twai_message_t *message, uint32_t timeoutMs = 100);

    bool isRunning() { return _running; }

    // Set CAN filter
    void setFilter(uint32_t id, uint32_t mask);
    void clearFilter();

private:
    gpio_num_t _txPin;
    gpio_num_t _rxPin;
    bool _running;

    // ISO-TP internals
    bool sendSingleFrame(uint32_t id, const uint8_t *data, uint16_t len);
    bool sendFirstFrame(uint32_t id, const uint8_t *data, uint16_t totalLen);
    bool sendConsecutiveFrames(uint32_t id, const uint8_t *data, uint16_t totalLen,
                               uint8_t blockSize, uint8_t separationTime);
    bool waitForFlowControl(uint32_t rxId, uint8_t *blockSize, uint8_t *separationTime);
    bool receiveISOTPResponse(uint32_t rxId, uint8_t *response,
                              uint16_t *responseLen, uint16_t maxResponseLen);
};

#endif
