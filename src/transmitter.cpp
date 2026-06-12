// ESP32 WROVER-E LoRa Transmitter using RYLR998 Module
// Hardware Serial2 is used for LoRa communication
#include <Arduino.h>
#include "transmitter.h"
#include "verbose.h"
HardwareSerial loraSerial(2);

// Pin definitions for ESP32 WROVER-E
// Using GPIO pins that are available and not used for PSRAM/Flash
#define LORA_RX_PIN 21  // Connect to RYLR998 TX
#define LORA_TX_PIN 22  // Connect to RYLR998 RX
//#define LORA_RST_PIN 19  // Optional: Connect to RYLR998 RST pin

// These values must match the receiver/Raspberry Pi side.
#define LORA_BAUD_RATE 115200
#define LORA_BAND 915000000
#define NETWORK_ID 1
#define DEVICE_ADDRESS 6
#define TARGET_ADDRESS 2

String readModuleResponse(uint32_t timeoutMs = 1000);
bool sendATCommand(String command, uint32_t timeoutMs = 1000);
bool sendMessage(int address, String message);
void checkForMessages();

static String sRxLine;
static bool sDeployCommandPending = false;

static bool isDeployCommand(const String& payload)
{
  String cmd = payload;
  cmd.trim();
  cmd.toUpperCase();
  return cmd == "DEPLOY_AIRBRAKE" || cmd == "DEPLOY" || cmd == "AIRBRAKE_DEPLOY";
}

static bool parseRcvFrame(const String& frame, String& senderAddress, String& messageData, String& rssi, String& snr)
{
  if (!frame.startsWith("+RCV=")) {
    return false;
  }

  int firstComma = frame.indexOf(',');
  int secondComma = frame.indexOf(',', firstComma + 1);
  int thirdComma = frame.indexOf(',', secondComma + 1);
  int fourthComma = frame.indexOf(',', thirdComma + 1);

  if (firstComma <= 0 || secondComma <= 0 || thirdComma <= 0 || fourthComma <= 0) {
    return false;
  }

  senderAddress = frame.substring(5, firstComma);
  messageData = frame.substring(secondComma + 1, thirdComma);
  rssi = frame.substring(thirdComma + 1, fourthComma);
  snr = frame.substring(fourthComma + 1);
  senderAddress.trim();
  messageData.trim();
  rssi.trim();
  snr.trim();
  return true;
}

static void handleIncomingLine(const String& line)
{
  if (line.length() == 0) {
    return;
  }

  String senderAddress;
  String messageData;
  String rssi;
  String snr;

  if (!parseRcvFrame(line, senderAddress, messageData, rssi, snr)) {
    return;
  }

  VLOG("Received message: " + line);
  VLOG("From: " + senderAddress);
  VLOG("Message: " + messageData);
  VLOG("RSSI: " + rssi + " dBm");
  VLOG("SNR: " + snr + " dB");

  if (isDeployCommand(messageData)) {
    sDeployCommandPending = true;
    Serial.println("[RXCMD] Deploy airbrake command detected");
  }
}

static void pumpIncoming(uint32_t budgetMs)
{
  uint32_t startMs = millis();

  while (true) {
    bool consumedAny = false;
    while (loraSerial.available() > 0) {
      consumedAny = true;
      char c = static_cast<char>(loraSerial.read());

      if (c == '\r') {
        continue;
      }

      if (c == '\n') {
        sRxLine.trim();
        handleIncomingLine(sRxLine);
        sRxLine = "";
      } else {
        sRxLine += c;
      }
    }

    if (budgetMs == 0) {
      break;
    }

    if (millis() - startMs >= budgetMs) {
      break;
    }

    if (!consumedAny) {
      delay(1);
    }
  }
}

bool transmitterInit() {
  VLOG("ESP32 WROVER-E LoRa Transmitter Starting...");

  // Optional: Hardware reset of LoRa module
  #ifdef LORA_RST_PIN
    pinMode(LORA_RST_PIN, OUTPUT);
    digitalWrite(LORA_RST_PIN, LOW);
    delay(100);
    digitalWrite(LORA_RST_PIN, HIGH);
    delay(1000);
  #endif

  // Initialize hardware serial for LoRa module
  // ESP32 WROVER-E: Using GPIO 21/22 which are available and safe to use
  loraSerial.begin(LORA_BAUD_RATE, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(1000);

  // Clear any stale bytes before issuing setup commands.
  while (loraSerial.available()) {
    loraSerial.read();
  }

  // Test communication with LoRa module
  VLOG("Testing LoRa module communication...");
  if (!sendATCommand("AT")) {
    return false;
  }

  // Configure LoRa module
  VLOG("Configuring LoRa module...");
  
  // Set frequency band (915MHz for US, change to 868000000 for EU)
  // Check your local regulations!
  if (!sendATCommand("AT+BAND=" + String(LORA_BAND))) {
    return false;
  }
  
  // Set network ID (0-15) to avoid interference
  if (!sendATCommand("AT+NETWORKID=" + String(NETWORK_ID))) {
    return false;
  }
  
  // Set this module's address
  if (!sendATCommand("AT+ADDRESS=" + String(DEVICE_ADDRESS))) {
    return false;
  }
  
  // Optional: Set transmission power (5-22 dBm)
  if (!sendATCommand("AT+PARAMETER=9,7,1,12")) {
    return false;
  }

  // Print back key config so mismatches are obvious.
  sendATCommand("AT+BAND?");
  sendATCommand("AT+NETWORKID?");
  sendATCommand("AT+ADDRESS?");
  sendATCommand("AT+PARAMETER?");
  
  VLOG("LoRa Module configured successfully!");
  return true;
}

bool transmitterSend(const String &payload) {
  return sendMessage(TARGET_ADDRESS, payload);
}

void transmitterPoll() {
  pumpIncoming(0);
}

bool transmitterReceiveDeployCommandWindow(uint32_t windowMs) {
  pumpIncoming(windowMs);

  if (!sDeployCommandPending) {
    return false;
  }

  sDeployCommandPending = false;
  return true;
}

String readModuleResponse(uint32_t timeoutMs) {
  String response;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (loraSerial.available()) {
      char c = static_cast<char>(loraSerial.read());
      if (c != '\r') {
        response += c;
      }
    }
    if (response.indexOf('\n') >= 0) {
      break;
    }
    delay(5);
  }

  response.trim();
  return response;
}

bool sendATCommand(String command, uint32_t timeoutMs) {
  VLOG("Sending: " + command);
  loraSerial.println(command);
  String response = readModuleResponse(timeoutMs);

  if (response.length() == 0) {
    VLOG("Response: [no reply]");
    return false;
  }

  VLOG("Response: " + response);
  return response.indexOf("+OK") >= 0 || response == "OK";
}

bool sendMessage(int address, String message) {
  // AT command format: AT+SEND=[Address],[Payload Length],[Payload]
  String atCommand = "AT+SEND=" + String(address) + "," + String(message.length()) + "," + message;
  
  // Serial.println("Transmitting to address " + String(address) + ": " + message);
  loraSerial.println(atCommand);
  
  String response = readModuleResponse(1200);
  if (response.length() == 0) {
    Serial.println("Transmission failed: no module response");
    return false;
  }

  if (response.indexOf("+OK") >= 0 || response == "OK") {
    // Serial.println("Message sent successfully!");
    return true;
  } else {
    Serial.println("Transmission failed: " + response);
    return false;
  }
}

void checkForMessages() {
  pumpIncoming(0);
}

// Optional: Function to put ESP32 into deep sleep to save power
void enterDeepSleep(int seconds) {
  Serial.println("Entering deep sleep for " + String(seconds) + " seconds...");
  Serial.flush();
  esp_sleep_enable_timer_wakeup(seconds * 1000000ULL); // Convert to microseconds
  esp_deep_sleep_start();
}