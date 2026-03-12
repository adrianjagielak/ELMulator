/**
 * WaveShare ESP32-S3-RS485-CAN — ELM327 WiFi-to-CAN Bridge
 *
 * Turns the WaveShare ESP32-S3-RS485-CAN board into a real ELM327-compatible
 * WiFi adapter. Diagnostic tools (PyRen, Torque, etc.) connect over WiFi and
 * send standard ELM327/OBD-II commands. This sketch forwards them as ISO-TP
 * frames on the CAN bus and returns the ECU responses.
 *
 * Board: WaveShare ESP32-S3-RS485-CAN
 *   CAN TX: GPIO15
 *   CAN RX: GPIO16
 *   USB-C:  for power + Serial debug output
 *
 * WiFi AP mode:
 *   SSID: "OBDII"  (no password)
 *   IP:   192.168.0.10
 *   Port: 35000
 *
 * Usage with PyRen:
 *   1. Flash this sketch to the WaveShare board
 *   2. Connect CAN_H and CAN_L to the OBD-II port (pins 6 & 14)
 *   3. Power the board via USB or OBD-II pin 16 (12V -> regulator)
 *   4. Connect your PC/phone WiFi to "OBDII"
 *   5. In PyRen, select WiFi ELM327 adapter at 192.168.0.10:35000
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include "CANBus.h"

// --- Configuration ---
const char *AP_SSID = "OBDII";
const char *AP_PASS = "";  // Open network; set a password if desired
const uint16_t WIFI_PORT = 35000;
const IPAddress AP_IP(192, 168, 0, 10);
const IPAddress AP_GW(192, 168, 0, 10);
const IPAddress AP_SUBNET(255, 255, 255, 0);

// CAN bus default: 500 kbps, 11-bit IDs (ISO 15765-4 standard OBD-II)
uint32_t canBaudRate = 500000;
bool canExtendedIds = false;

// Default OBD-II arbitration IDs (11-bit)
uint32_t canTxId = 0x7DF;  // Broadcast request
uint32_t canRxId = 0x7E8;  // ECU response (first ECU)
uint32_t canCustomHeader = 0;
bool useCustomHeader = false;

// ELM327 state
bool echoEnabled = true;
bool lineFeedsEnabled = true;
bool spacesEnabled = true;
bool headersEnabled = false;
char currentProtocol = '6';  // ISO 15765-4 CAN 500k 11-bit
bool canAutoFormat = true;
uint8_t canTimeout = 0x19;  // Default ~100ms (0x19 * 4ms)
bool canStarted = false;

// --- Globals ---
WiFiServer server(WIFI_PORT);
WiFiClient client;
CANBus canbus(CAN_TX_PIN, CAN_RX_PIN);

// Buffers
char cmdBuffer[256];
uint8_t cmdLen = 0;
uint8_t canResponseBuf[ISOTP_MAX_DATA_SIZE];

// --- Forward declarations ---
void handleCommand(const char *cmd);
void handleATCommand(const char *cmd);
void handleDiagCommand(const char *cmd);
void sendResponse(const char *resp);
void sendPrompt();
void sendOK();
void sendError();
void sendNoData();
void sendUnknown();
void ensureCANStarted();
uint8_t hexCharToNibble(char c);
bool parseHexString(const char *hex, uint8_t *bytes, uint16_t *len, uint16_t maxLen);
void formatResponseHex(const uint8_t *data, uint16_t len, char *out, bool withSpaces);

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("=== WaveShare ESP32-S3 ELM327 CAN Bridge ===");
    Serial.printf("CAN TX: GPIO%d, CAN RX: GPIO%d\n", CAN_TX_PIN, CAN_RX_PIN);

    // Start WiFi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GW, AP_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("WiFi AP: %s @ %s:%d\n", AP_SSID, WiFi.softAPIP().toString().c_str(), WIFI_PORT);

    server.begin();
    Serial.println("Waiting for client connection...");
}

void loop()
{
    // Accept new client
    if (!client || !client.connected())
    {
        client = server.available();
        if (client)
        {
            Serial.println("Client connected");
            cmdLen = 0;
        }
    }

    if (!client || !client.connected())
        return;

    // Read data from client
    while (client.available())
    {
        char c = client.read();

        // Carriage return = end of command
        if (c == '\r' || c == '\n')
        {
            if (cmdLen > 0)
            {
                cmdBuffer[cmdLen] = '\0';

                // Echo if enabled
                if (echoEnabled)
                {
                    client.print(cmdBuffer);
                    client.print("\r");
                }

                // Process
                handleCommand(cmdBuffer);
                cmdLen = 0;
            }
        }
        else if (cmdLen < sizeof(cmdBuffer) - 1)
        {
            cmdBuffer[cmdLen++] = c;
        }
    }
}

void ensureCANStarted()
{
    if (!canStarted)
    {
        canStarted = canbus.begin(canBaudRate);
        if (canStarted)
            Serial.println("CAN bus initialized");
        else
            Serial.println("CAN bus init FAILED");
    }
}

// --- Command Router ---

void handleCommand(const char *cmd)
{
    // Skip leading whitespace
    while (*cmd == ' ')
        cmd++;

    if (strlen(cmd) == 0)
    {
        sendPrompt();
        return;
    }

    // Uppercase copy for matching
    char upper[256];
    strncpy(upper, cmd, sizeof(upper) - 1);
    upper[sizeof(upper) - 1] = '\0';
    for (char *p = upper; *p; p++)
        *p = toupper(*p);

    // Remove spaces for matching
    char noSpaces[256];
    int j = 0;
    for (int i = 0; upper[i]; i++)
    {
        if (upper[i] != ' ')
            noSpaces[j++] = upper[i];
    }
    noSpaces[j] = '\0';

    if (strncmp(noSpaces, "AT", 2) == 0)
    {
        handleATCommand(noSpaces + 2);
    }
    else
    {
        handleDiagCommand(noSpaces);
    }
}

// --- AT Command Handler ---

void handleATCommand(const char *cmd)
{
    // AT Z - Reset
    if (strcmp(cmd, "Z") == 0)
    {
        echoEnabled = true;
        lineFeedsEnabled = true;
        spacesEnabled = true;
        headersEnabled = false;
        canAutoFormat = true;
        useCustomHeader = false;
        canTxId = 0x7DF;
        canRxId = 0x7E8;
        canTimeout = 0x19;
        sendResponse("\r\rELM327 v1.5");
        sendPrompt();
        return;
    }

    // AT D - Set defaults
    if (strcmp(cmd, "D") == 0)
    {
        echoEnabled = true;
        lineFeedsEnabled = true;
        spacesEnabled = true;
        headersEnabled = false;
        useCustomHeader = false;
        canTxId = 0x7DF;
        canRxId = 0x7E8;
        sendOK();
        return;
    }

    // AT I - Print version
    if (strcmp(cmd, "I") == 0)
    {
        sendResponse("ELM327 v1.5");
        sendPrompt();
        return;
    }

    // AT @1 - Device description
    if (strcmp(cmd, "@1") == 0)
    {
        sendResponse("ELMulator CAN Bridge - WaveShare ESP32-S3");
        sendPrompt();
        return;
    }

    // AT E0/E1 - Echo off/on
    if (strcmp(cmd, "E0") == 0) { echoEnabled = false; sendOK(); return; }
    if (strcmp(cmd, "E1") == 0) { echoEnabled = true; sendOK(); return; }

    // AT L0/L1 - Linefeeds off/on
    if (strcmp(cmd, "L0") == 0) { lineFeedsEnabled = false; sendOK(); return; }
    if (strcmp(cmd, "L1") == 0) { lineFeedsEnabled = true; sendOK(); return; }

    // AT S0/S1 - Spaces off/on
    if (strcmp(cmd, "S0") == 0) { spacesEnabled = false; sendOK(); return; }
    if (strcmp(cmd, "S1") == 0) { spacesEnabled = true; sendOK(); return; }

    // AT H0/H1 - Headers off/on
    if (strcmp(cmd, "H0") == 0) { headersEnabled = false; sendOK(); return; }
    if (strcmp(cmd, "H1") == 0) { headersEnabled = true; sendOK(); return; }

    // AT M0/M1 - Memory off/on (accepted, no effect)
    if (strcmp(cmd, "M0") == 0 || strcmp(cmd, "M1") == 0) { sendOK(); return; }

    // AT CAF0/CAF1 - CAN auto format
    if (strcmp(cmd, "CAF0") == 0) { canAutoFormat = false; sendOK(); return; }
    if (strcmp(cmd, "CAF1") == 0) { canAutoFormat = true; sendOK(); return; }

    // AT CFC0/CFC1 - CAN flow control
    if (strcmp(cmd, "CFC0") == 0 || strcmp(cmd, "CFC1") == 0) { sendOK(); return; }

    // AT CSM0/CSM1 - CAN silent mode
    if (strcmp(cmd, "CSM0") == 0 || strcmp(cmd, "CSM1") == 0) { sendOK(); return; }

    // AT AT0/AT1/AT2 - Adaptive timing
    if (strncmp(cmd, "AT", 2) == 0) { sendOK(); return; }

    // AT SP x - Set protocol
    if (strncmp(cmd, "SP", 2) == 0)
    {
        char proto = cmd[2];
        if (proto == 'A' && cmd[3]) proto = cmd[3]; // AT SP Ax
        currentProtocol = proto;
        // Adjust baud rate based on protocol
        switch (proto)
        {
        case '6': canBaudRate = 500000; canExtendedIds = false; break;
        case '7': canBaudRate = 500000; canExtendedIds = true; break;
        case '8': canBaudRate = 250000; canExtendedIds = false; break;
        case '9': canBaudRate = 250000; canExtendedIds = true; break;
        default: break;
        }
        // Restart CAN with new settings if already running
        if (canStarted)
        {
            canbus.stop();
            canStarted = false;
        }
        sendOK();
        return;
    }

    // AT TP x - Try protocol
    if (strncmp(cmd, "TP", 2) == 0)
    {
        // Same as SP for our purposes
        char proto = cmd[2];
        if (proto == 'A' && cmd[3]) proto = cmd[3];
        currentProtocol = proto;
        switch (proto)
        {
        case '6': canBaudRate = 500000; canExtendedIds = false; break;
        case '7': canBaudRate = 500000; canExtendedIds = true; break;
        case '8': canBaudRate = 250000; canExtendedIds = false; break;
        case '9': canBaudRate = 250000; canExtendedIds = true; break;
        default: break;
        }
        if (canStarted)
        {
            canbus.stop();
            canStarted = false;
        }
        sendOK();
        return;
    }

    // AT DPN - Display protocol number
    if (strcmp(cmd, "DPN") == 0)
    {
        char resp[4];
        snprintf(resp, sizeof(resp), "%c", currentProtocol);
        sendResponse(resp);
        sendPrompt();
        return;
    }

    // AT DP - Display protocol
    if (strcmp(cmd, "DP") == 0)
    {
        sendResponse("ISO 15765-4 (CAN 11/500)");
        sendPrompt();
        return;
    }

    // AT SH xxx - Set header (CAN TX ID)
    if (strncmp(cmd, "SH", 2) == 0)
    {
        const char *hexStr = cmd + 2;
        uint32_t header = strtoul(hexStr, NULL, 16);
        canTxId = header;
        // Auto-calculate expected response ID
        if (header >= 0x700 && header < 0x800)
        {
            canRxId = header + 8;
        }
        else if (header == 0x7DF)
        {
            canRxId = 0x7E8; // Broadcast -> first ECU
        }
        useCustomHeader = true;
        Serial.printf("CAN Header set: TX=0x%03X RX=0x%03X\n", canTxId, canRxId);
        sendOK();
        return;
    }

    // AT CF xxx - Set CAN filter
    if (strncmp(cmd, "CF", 2) == 0) { sendOK(); return; }

    // AT CM xxx - Set CAN mask
    if (strncmp(cmd, "CM", 2) == 0) { sendOK(); return; }

    // AT FC - Flow control commands
    if (strncmp(cmd, "FC", 2) == 0) { sendOK(); return; }

    // AT ST xx - Set timeout
    if (strncmp(cmd, "ST", 2) == 0)
    {
        canTimeout = strtoul(cmd + 2, NULL, 16);
        if (canTimeout == 0) canTimeout = 0x19;
        sendOK();
        return;
    }

    // AT RV - Read voltage
    if (strcmp(cmd, "RV") == 0)
    {
        sendResponse("12.6V");
        sendPrompt();
        return;
    }

    // AT PC - Protocol close
    if (strcmp(cmd, "PC") == 0) { sendOK(); return; }

    // AT WS - Warm start (same as reset for us)
    if (strcmp(cmd, "WS") == 0)
    {
        sendResponse("\r\rELM327 v1.5");
        sendPrompt();
        return;
    }

    // AT AR - Auto receive
    if (strcmp(cmd, "AR") == 0) { sendOK(); return; }

    // AT AL/NL - Allow long/normal messages
    if (strcmp(cmd, "AL") == 0 || strcmp(cmd, "NL") == 0) { sendOK(); return; }

    // AT BI - Bypass init
    if (strcmp(cmd, "BI") == 0) { sendOK(); return; }

    // AT D0/D1 - Display DLC
    if (strcmp(cmd, "D0") == 0 || strcmp(cmd, "D1") == 0) { sendOK(); return; }

    // AT V0/V1 - Variable DLC
    if (strcmp(cmd, "V0") == 0 || strcmp(cmd, "V1") == 0) { sendOK(); return; }

    // AT CEA - CAN extended addressing
    if (strncmp(cmd, "CEA", 3) == 0) { sendOK(); return; }

    // AT R0/R1 - Responses off/on
    if (strcmp(cmd, "R0") == 0 || strcmp(cmd, "R1") == 0) { sendOK(); return; }

    // AT SR / RA - Set receive address
    if (strncmp(cmd, "SR", 2) == 0 || strncmp(cmd, "RA", 2) == 0)
    {
        const char *hexStr = cmd + 2;
        canRxId = strtoul(hexStr, NULL, 16);
        Serial.printf("CAN RX ID set: 0x%03X\n", canRxId);
        sendOK();
        return;
    }

    // AT MA - Monitor all
    if (strcmp(cmd, "MA") == 0)
    {
        ensureCANStarted();
        sendResponse("Monitoring...");
        // Monitor for a few seconds
        twai_message_t msg;
        unsigned long start = millis();
        while (millis() - start < 5000)
        {
            if (canbus.receiveAny(&msg, 100))
            {
                char line[64];
                int pos = 0;
                pos += snprintf(line + pos, sizeof(line) - pos, "%03X ", (unsigned int)msg.identifier);
                for (int i = 0; i < msg.data_length_code; i++)
                    pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", msg.data[i]);
                sendResponse(line);
            }
            if (client.available()) break; // Any key press stops monitoring
        }
        sendPrompt();
        return;
    }

    // AT PB - Set protocol B options
    if (strncmp(cmd, "PB", 2) == 0) { sendOK(); return; }

    // AT SS - Standard search order
    if (strcmp(cmd, "SS") == 0) { sendOK(); return; }

    // AT SW - Set wakeup
    if (strncmp(cmd, "SW", 2) == 0) { sendOK(); return; }

    // AT TA - Set tester address
    if (strncmp(cmd, "TA", 2) == 0) { sendOK(); return; }

    // Unknown AT command - accept it silently (many tools probe for features)
    Serial.printf("Unknown AT: %s\n", cmd);
    sendOK();
}

// --- Diagnostic Command Handler ---
// Receives hex string from the client, sends it as ISO-TP on CAN, returns response

void handleDiagCommand(const char *cmd)
{
    // Validate hex string
    for (const char *p = cmd; *p; p++)
    {
        if (!isxdigit(*p))
        {
            sendUnknown();
            return;
        }
    }

    uint16_t cmdHexLen = strlen(cmd);
    if (cmdHexLen < 2 || cmdHexLen % 2 != 0)
    {
        // Try to handle odd-length commands (some tools send "0100" as "100")
        // If odd, might be missing leading zero - but usually it's even
        if (cmdHexLen % 2 != 0)
        {
            sendUnknown();
            return;
        }
    }

    // Parse hex string to bytes
    uint8_t txData[128];
    uint16_t txLen = 0;
    if (!parseHexString(cmd, txData, &txLen, sizeof(txData)))
    {
        sendUnknown();
        return;
    }

    // Make sure CAN is running
    ensureCANStarted();
    if (!canbus.isRunning())
    {
        sendResponse("CAN ERROR");
        sendPrompt();
        return;
    }

    // Calculate timeout from AT ST setting
    uint32_t timeoutMs = (uint32_t)canTimeout * 4;
    if (timeoutMs < 100) timeoutMs = 100;

    Serial.printf("TX [0x%03X]: ", canTxId);
    for (int i = 0; i < txLen; i++) Serial.printf("%02X ", txData[i]);
    Serial.println();

    // Send ISO-TP request and get response
    uint16_t rxLen = 0;
    bool gotResponse = canbus.sendISOTP(canTxId, canRxId,
                                         txData, txLen,
                                         canResponseBuf, &rxLen,
                                         sizeof(canResponseBuf));

    if (!gotResponse || rxLen == 0)
    {
        // Try to collect responses from multiple ECUs if using broadcast (0x7DF)
        if (canTxId == 0x7DF)
        {
            // Already tried and failed
            sendNoData();
            return;
        }
        sendNoData();
        return;
    }

    Serial.printf("RX [0x%03X] %d bytes: ", canRxId, rxLen);
    for (int i = 0; i < rxLen; i++) Serial.printf("%02X ", canResponseBuf[i]);
    Serial.println();

    // Format response
    char responseStr[ISOTP_MAX_DATA_SIZE * 3 + 16];
    int pos = 0;

    // Add header if enabled
    if (headersEnabled)
    {
        if (spacesEnabled)
            pos += snprintf(responseStr + pos, sizeof(responseStr) - pos, "%03X ", canRxId);
        else
            pos += snprintf(responseStr + pos, sizeof(responseStr) - pos, "%03X", canRxId);
    }

    // Format data bytes
    for (uint16_t i = 0; i < rxLen; i++)
    {
        pos += snprintf(responseStr + pos, sizeof(responseStr) - pos, "%02X", canResponseBuf[i]);
        if (spacesEnabled && i < rxLen - 1)
            pos += snprintf(responseStr + pos, sizeof(responseStr) - pos, " ");
    }

    sendResponse(responseStr);
    sendPrompt();
}

// --- Response helpers ---

void sendResponse(const char *resp)
{
    client.print("\r");
    if (lineFeedsEnabled)
        client.print("\n");
    client.print(resp);
}

void sendPrompt()
{
    client.print("\r");
    if (lineFeedsEnabled)
        client.print("\n");
    client.print(">");
    client.flush();
}

void sendOK()
{
    sendResponse("OK");
    sendPrompt();
}

void sendError()
{
    sendResponse("ERROR");
    sendPrompt();
}

void sendNoData()
{
    sendResponse("NO DATA");
    sendPrompt();
}

void sendUnknown()
{
    sendResponse("?");
    sendPrompt();
}

// --- Hex utilities ---

uint8_t hexCharToNibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

bool parseHexString(const char *hex, uint8_t *bytes, uint16_t *len, uint16_t maxLen)
{
    *len = 0;
    uint16_t hexLen = strlen(hex);
    if (hexLen % 2 != 0) return false;

    for (uint16_t i = 0; i < hexLen && *len < maxLen; i += 2)
    {
        if (!isxdigit(hex[i]) || !isxdigit(hex[i + 1]))
            return false;
        bytes[(*len)++] = (hexCharToNibble(hex[i]) << 4) | hexCharToNibble(hex[i + 1]);
    }
    return true;
}
