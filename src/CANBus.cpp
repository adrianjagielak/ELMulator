#include "CANBus.h"

CANBus::CANBus(gpio_num_t txPin, gpio_num_t rxPin)
    : _txPin(txPin), _rxPin(rxPin), _running(false)
{
}

CANBus::~CANBus()
{
    stop();
}

bool CANBus::begin(uint32_t baudRate)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 8;

    twai_timing_config_t t_config;
    switch (baudRate)
    {
    case 125000:
        t_config = TWAI_TIMING_CONFIG_125KBITS();
        break;
    case 250000:
        t_config = TWAI_TIMING_CONFIG_250KBITS();
        break;
    case 500000:
    default:
        t_config = TWAI_TIMING_CONFIG_500KBITS();
        break;
    case 800000:
        t_config = TWAI_TIMING_CONFIG_800KBITS();
        break;
    case 1000000:
        t_config = TWAI_TIMING_CONFIG_1MBITS();
        break;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        Serial.println("TWAI driver install failed");
        return false;
    }

    if (twai_start() != ESP_OK)
    {
        Serial.println("TWAI start failed");
        twai_driver_uninstall();
        return false;
    }

    _running = true;
    Serial.println("CAN bus started");
    return true;
}

void CANBus::stop()
{
    if (_running)
    {
        twai_stop();
        twai_driver_uninstall();
        _running = false;
    }
}

bool CANBus::sendFrame(uint32_t id, const uint8_t *data, uint8_t len, bool extendedId)
{
    if (!_running)
        return false;

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier = id;
    msg.extd = extendedId ? 1 : 0;
    msg.data_length_code = len;
    memcpy(msg.data, data, len);

    return twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK;
}

bool CANBus::receiveFrame(twai_message_t *message, uint32_t timeoutMs)
{
    if (!_running)
        return false;
    return twai_receive(message, pdMS_TO_TICKS(timeoutMs)) == ESP_OK;
}

bool CANBus::receiveAny(twai_message_t *message, uint32_t timeoutMs)
{
    return receiveFrame(message, timeoutMs);
}

void CANBus::setFilter(uint32_t id, uint32_t mask)
{
    // Filter can only be set before start; for runtime filtering we do it in software
    // This is noted as a limitation - TWAI filter must be set before driver start
}

void CANBus::clearFilter()
{
    // Same limitation as above
}

// --- ISO-TP Implementation ---

bool CANBus::sendSingleFrame(uint32_t id, const uint8_t *data, uint16_t len)
{
    uint8_t frame[8] = {0};
    frame[0] = ISOTP_SINGLE_FRAME | (len & 0x0F);
    memcpy(&frame[1], data, len);
    return sendFrame(id, frame, 8);
}

bool CANBus::sendFirstFrame(uint32_t id, const uint8_t *data, uint16_t totalLen)
{
    uint8_t frame[8] = {0};
    frame[0] = ISOTP_FIRST_FRAME | ((totalLen >> 8) & 0x0F);
    frame[1] = totalLen & 0xFF;
    memcpy(&frame[2], data, 6); // First 6 bytes of data
    return sendFrame(id, frame, 8);
}

bool CANBus::sendConsecutiveFrames(uint32_t id, const uint8_t *data, uint16_t totalLen,
                                    uint8_t blockSize, uint8_t separationTime)
{
    uint16_t offset = 6; // First 6 bytes already sent in First Frame
    uint8_t seqNum = 1;
    uint8_t blockCount = 0;

    while (offset < totalLen)
    {
        uint8_t frame[8] = {0};
        frame[0] = ISOTP_CONSECUTIVE_FRAME | (seqNum & 0x0F);
        uint16_t remaining = totalLen - offset;
        uint8_t copyLen = (remaining > 7) ? 7 : remaining;
        memcpy(&frame[1], &data[offset], copyLen);

        if (!sendFrame(id, frame, 8))
            return false;

        offset += copyLen;
        seqNum = (seqNum + 1) & 0x0F;
        blockCount++;

        // Respect separation time
        if (separationTime > 0 && separationTime <= 127)
        {
            delay(separationTime);
        }
        else
        {
            delay(ISOTP_CF_INTERVAL_MS);
        }

        // If block size reached (and not 0 = unlimited), wait for next FC
        if (blockSize > 0 && blockCount >= blockSize && offset < totalLen)
        {
            uint8_t newBS, newST;
            if (!waitForFlowControl(id + 8, &newBS, &newST)) // Response ID is typically request + 8
                return false;
            blockSize = newBS;
            separationTime = newST;
            blockCount = 0;
        }
    }
    return true;
}

bool CANBus::waitForFlowControl(uint32_t rxId, uint8_t *blockSize, uint8_t *separationTime)
{
    twai_message_t msg;
    uint32_t start = millis();

    while (millis() - start < CAN_RESPONSE_TIMEOUT_MS)
    {
        if (receiveFrame(&msg, 100))
        {
            if (msg.identifier == rxId && (msg.data[0] & 0xF0) == ISOTP_FLOW_CONTROL)
            {
                uint8_t flowStatus = msg.data[0] & 0x0F;
                if (flowStatus == 0) // Continue To Send
                {
                    *blockSize = msg.data[1];
                    *separationTime = msg.data[2];
                    return true;
                }
                else if (flowStatus == 1) // Wait
                {
                    delay(ISOTP_FC_WAIT_MS);
                    continue;
                }
                else // Overflow/abort
                {
                    return false;
                }
            }
        }
    }
    return false;
}

bool CANBus::receiveISOTPResponse(uint32_t rxId, uint8_t *response,
                                   uint16_t *responseLen, uint16_t maxResponseLen)
{
    twai_message_t msg;
    uint32_t start = millis();

    // Wait for first response frame
    while (millis() - start < CAN_RESPONSE_TIMEOUT_MS)
    {
        if (!receiveFrame(&msg, CAN_RESPONSE_TIMEOUT_MS - (millis() - start)))
            return false;

        if (msg.identifier != rxId)
            continue;

        uint8_t frameType = msg.data[0] & 0xF0;

        if (frameType == ISOTP_SINGLE_FRAME)
        {
            uint8_t len = msg.data[0] & 0x0F;
            if (len > maxResponseLen)
                return false;
            memcpy(response, &msg.data[1], len);
            *responseLen = len;
            return true;
        }
        else if (frameType == ISOTP_FIRST_FRAME)
        {
            uint16_t totalLen = ((msg.data[0] & 0x0F) << 8) | msg.data[1];
            if (totalLen > maxResponseLen)
                return false;

            memcpy(response, &msg.data[2], 6);
            uint16_t received = 6;
            uint8_t expectedSeq = 1;

            // Send Flow Control - Continue To Send, no block limit, min separation
            uint8_t fc[8] = {ISOTP_FLOW_CONTROL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            // Send FC with the request ID (rxId - 8 is typically the request ID)
            sendFrame(rxId - 8, fc, 8);

            // Receive consecutive frames
            while (received < totalLen)
            {
                if (!receiveFrame(&msg, CAN_RESPONSE_TIMEOUT_MS))
                    return false;

                if (msg.identifier != rxId)
                    continue;

                if ((msg.data[0] & 0xF0) != ISOTP_CONSECUTIVE_FRAME)
                    continue;

                uint8_t seqNum = msg.data[0] & 0x0F;
                if (seqNum != expectedSeq)
                {
                    Serial.printf("ISO-TP sequence error: expected %d got %d\n", expectedSeq, seqNum);
                    return false;
                }

                uint16_t remaining = totalLen - received;
                uint8_t copyLen = (remaining > 7) ? 7 : remaining;
                memcpy(&response[received], &msg.data[1], copyLen);
                received += copyLen;
                expectedSeq = (expectedSeq + 1) & 0x0F;
            }

            *responseLen = totalLen;
            return true;
        }
    }

    return false;
}

bool CANBus::sendISOTP(uint32_t txId, uint32_t rxId,
                        const uint8_t *data, uint16_t dataLen,
                        uint8_t *response, uint16_t *responseLen,
                        uint16_t maxResponseLen)
{
    if (!_running)
        return false;

    // Send request
    if (dataLen <= 7)
    {
        if (!sendSingleFrame(txId, data, dataLen))
            return false;
    }
    else
    {
        if (!sendFirstFrame(txId, data, dataLen))
            return false;

        uint8_t blockSize, separationTime;
        if (!waitForFlowControl(rxId, &blockSize, &separationTime))
            return false;

        if (!sendConsecutiveFrames(txId, data, dataLen, blockSize, separationTime))
            return false;
    }

    // Receive response
    return receiveISOTPResponse(rxId, response, responseLen, maxResponseLen);
}
