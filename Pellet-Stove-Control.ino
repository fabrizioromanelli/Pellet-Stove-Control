#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

#define mqtt_server ""
#define mqtt_port 1883
#define mqtt_topic ""
#define mqtt_user ""
#define mqtt_pass ""

unsigned long UPDATE_PERIOD = 1800000; // 30m (in ms)
unsigned long FAST_UPDATE_PERIOD = 60000; // 1m (in ms)

#define FAST_LOOP_CYCLES 30
#define RESET_CYCLES 60

SoftwareSerial StoveSerial;
#define SERIAL_MODE SWSERIAL_8N2
#define RX_PIN D3
#define TX_PIN D4
#define ENABLE_RX D2
#define PIN_RESET D0

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wm;

static volatile bool sem = false;
bool fastUpdate = false;
bool boot = true;
static unsigned long loopCounter = 0;
static unsigned long resetCounter = 0;
unsigned long previousMillis = 0;

#define connection_topic mqtt_topic "/connection"
#define state_topic mqtt_topic "/state"
#define onoff_topic mqtt_topic "/onoff"
#define ambtemp_topic mqtt_topic "/ambtemp"
#define fumetemp_topic mqtt_topic "/fumetemp"
#define flame_topic mqtt_topic "/power"
#define fan_topic mqtt_topic "/fan"
#define tempset_topic mqtt_topic "/tempset"
#define cmd mqtt_topic "/cmd"

#define device_information "{\"manufacturer\": \"Fabrizio Romanelli\",\"identifiers\": [\"7a396f39-80d2-493b-8e8e-31a70e700bc6\"],\"model\": \"Micronova Controller\",\"name\": \"Micronova Controller\",\"sw_version\": \"1.0.0.0\"}"

#define _RAMR 0x00
#define _RAMW 0x80
#define stoveStateAddr 0x21
#define ambTempAddr 0x01
#define fumesTempAddr 0x3E
#define stovePOnOffAddr 0x58

#define _EEPROMR 0x20
#define _EEPROMW 0xA0
#define powerSetAddr 0x89
#define fanSetAddr 0x8A
#define tempSetAddr 0x8B

// Checksum: Code+Address+Value (hex)
const char stoveOnOff[4] = {_RAMW, stovePOnOffAddr, 0x5A, (char)(_RAMW + stovePOnOffAddr + 0x5A)};
const char stoveOff[4] = {_RAMW, stoveStateAddr, 0x00, (char)(_RAMW + stoveStateAddr + 0x00)};

const char stovePW[6][4] = {
{_EEPROMW, powerSetAddr, 0x00, (char)(_EEPROMW + powerSetAddr + 0x00)},
{_EEPROMW, powerSetAddr, 0x01, (char)(_EEPROMW + powerSetAddr + 0x01)},
{_EEPROMW, powerSetAddr, 0x02, (char)(_EEPROMW + powerSetAddr + 0x02)},
{_EEPROMW, powerSetAddr, 0x03, (char)(_EEPROMW + powerSetAddr + 0x03)},
{_EEPROMW, powerSetAddr, 0x04, (char)(_EEPROMW + powerSetAddr + 0x04)},
{_EEPROMW, powerSetAddr, 0x05, (char)(_EEPROMW + powerSetAddr + 0x05)}
};

const char stoveFN[6][4] = {
{_EEPROMW, fanSetAddr, 0x00, (char)(_EEPROMW + fanSetAddr + 0x00)},
{_EEPROMW, fanSetAddr, 0x01, (char)(_EEPROMW + fanSetAddr + 0x01)},
{_EEPROMW, fanSetAddr, 0x02, (char)(_EEPROMW + fanSetAddr + 0x02)},
{_EEPROMW, fanSetAddr, 0x03, (char)(_EEPROMW + fanSetAddr + 0x03)},
{_EEPROMW, fanSetAddr, 0x04, (char)(_EEPROMW + fanSetAddr + 0x04)},
{_EEPROMW, fanSetAddr, 0x05, (char)(_EEPROMW + fanSetAddr + 0x05)}
};

uint8_t stoveState, tempSet, fumesTemp, flamePower, fanSpeed;
float ambTemp;
char stoveRxData[2];  // When the stove is sending data, it sends two bytes: a checksum and the value

// Utility: non-blocking delay
void nonBlockingDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    client.loop();
    ArduinoOTA.handle();
    yield();
  }
}

void setSemaphore(bool value) {
  noInterrupts();
  sem = value;
  interrupts();
}

// Main function to send commands
void sendCommand(const char* _cmd) {
  if (!sem) {
    setSemaphore(true);
  for (int i = 0; i < 4; i++) {
    StoveSerial.write(_cmd[i]);
    nonBlockingDelay(1);
  }
  nonBlockingDelay(50);
  setSemaphore(false);
  }
}

void setup_wifi() // Setup WiFiManager and connect to WiFi
{
  ArduinoOTA.setHostname(mqtt_topic);
  ArduinoOTA.setPassword("micronova");
  ArduinoOTA.begin();
  WiFi.mode(WIFI_STA);
  wm.setConnectTimeout(30);
  wm.autoConnect(mqtt_topic);
}

void reconnect() {
  unsigned long startAttemptTime = millis();
  while (!client.connected() && millis() - startAttemptTime < 30000) { // 30 seconds timeout
    Serial.print("Attempting MQTT connection...");
    char clientId[15];
    snprintf(clientId, sizeof(clientId), "ESPClient-%X", random(0xffff));
    if (client.connect(clientId, mqtt_user, mqtt_pass)) {
      client.setBufferSize(1024);
      Serial.println("connected");
      client.subscribe(cmd);
      break;
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      nonBlockingDelay(5000); // Wait before retrying
    }
  }
}

// This is the callback from a topic arrived via MQTT
void callback(char *topic, byte *payload, unsigned int length) {
  if (length == 0) return;

  if (payload[0] == 'O' && length >= 2) {
    if (payload[1] == 'N') {
      sendCommand(stoveOnOff);
      client.publish(onoff_topic, "ON", true);
    } else if (payload[1] == 'F') {
      sendCommand(stoveOff);
      client.publish(onoff_topic, "OFF", true);
    }
    fastUpdate = true;
  }
  else if (payload[0] == 'P' && length == 2) {
    int idx = payload[1] - '0';
    if (idx >= 0 && idx <= 5) sendCommand(stovePW[idx]);
    fastUpdate = true;
  }
  else if (payload[0] == 'F' && length == 2) {
    int idx = payload[1] - '0';
    if (idx >= 0 && idx <= 5) sendCommand(stoveFN[idx]);
    fastUpdate = true;
  }
  else if (payload[0] == 'T' && length == 3) {
    char t[3] = {(char)payload[1], (char)payload[2], '\0'};
    char tempValue = (char)strtol(t, NULL, 10);
    const char temperatureSet[4] = {_EEPROMW, tempSetAddr, tempValue, (char)(_EEPROMW + tempSetAddr + tempValue)};
    sendCommand(temperatureSet);
    fastUpdate = true;
  }
  else if (payload[0] == 'E') {
    sendCommand(stoveOff);
    client.publish(onoff_topic, "OFF", true);
    fastUpdate = true;
  }
}

void checkStoveReply(unsigned long timeout = 200) {
  unsigned long start = millis();
  uint8_t rxCount = 0;
  stoveRxData[0] = 0x00;
  stoveRxData[1] = 0x00;

  while (millis() - start < timeout) {
    if (StoveSerial.available()) { // It has to be exactly 2 bytes, otherwise it's an error
      stoveRxData[rxCount++] = StoveSerial.read();
      if (rxCount == 2) break;
    }
    yield();
  }
  digitalWrite(ENABLE_RX, HIGH);

  if (rxCount == 2) {
    byte val = stoveRxData[1];
    byte checksum = stoveRxData[0];
    byte param = checksum - val;
    switch (param)
    {
      // RAM Cases
      case stoveStateAddr:
        stoveState = val;
        switch (stoveState)
        {
          case 0:
            client.publish(state_topic, "Spenta", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 1:
            client.publish(state_topic, "Avvio", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 2:
            client.publish(state_topic, "Caricamento pellet", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 3:
            client.publish(state_topic, "Accensione", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 4:
            client.publish(state_topic, "Accesa", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 5:
            client.publish(state_topic, "Pulizia braciere", true);
            break;
          case 6:
            client.publish(state_topic, "Pulizia finale", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 7:
            client.publish(state_topic, "Standby", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 8:
            client.publish(state_topic, "Pellet mancante", true);
            break;
          case 9:
            client.publish(state_topic, "Mancata accensione", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 10:
            client.publish(state_topic, "Allarme", true);
            break;
        }
        break;
      case ambTempAddr:
        ambTemp = (float)val / 2;
        char ambTempStr[10];
        dtostrf(ambTemp, 4, 2, ambTempStr);
        client.publish(ambtemp_topic, ambTempStr, true);
        break;
      case fumesTempAddr:
        fumesTemp = val;
        char fumesTempStr[10];
        sprintf(fumesTempStr, "%d", fumesTemp);
        client.publish(fumetemp_topic, fumesTempStr, true);
        break;
      // EEPROM Cases
      case tempSetAddr + _EEPROMR:
        tempSet = val;
        char tempSetStr[10];
        sprintf(tempSetStr, "%d", tempSet);
        client.publish(tempset_topic, tempSetStr, true);
        break;
      case powerSetAddr + _EEPROMR:
        flamePower = val;
        char flamePowerStr[10];
        sprintf(flamePowerStr, "%d", flamePower);
        client.publish(flame_topic, flamePowerStr, true);
        break;
      case fanSetAddr + _EEPROMR:
        fanSpeed = val;
        char fanSpeedStr[10];
        sprintf(fanSpeedStr, "%d", fanSpeed);
        client.publish(fan_topic, fanSpeedStr, true);
        break;
    }
  }
}

void getStoveState() // Get detailed stove state
{
  const byte readByte = _RAMR;
  StoveSerial.write(readByte);
  nonBlockingDelay(1);
  StoveSerial.write(stoveStateAddr);
  digitalWrite(ENABLE_RX, LOW);
  nonBlockingDelay(80);
  checkStoveReply();
}

void getAmbTemp() // Get room temperature
{
  const byte readByte = _RAMR;
  StoveSerial.write(readByte);
  nonBlockingDelay(1);
  StoveSerial.write(ambTempAddr);
  digitalWrite(ENABLE_RX, LOW);
  nonBlockingDelay(80);
  checkStoveReply();
}

void getFumeTemp() // Get flue gas temperature
{
  const byte readByte = _RAMR;
  StoveSerial.write(readByte);
  nonBlockingDelay(1);
  StoveSerial.write(fumesTempAddr);
  digitalWrite(ENABLE_RX, LOW);
  nonBlockingDelay(80);
  checkStoveReply();
}

void getFlamePower() // Get the flame power
{
  const byte readByte = _EEPROMR;
  StoveSerial.write(readByte);
  nonBlockingDelay(1);
  StoveSerial.write(powerSetAddr);
  digitalWrite(ENABLE_RX, LOW);
  nonBlockingDelay(80);
  checkStoveReply();
}

void getFanSpeed() // Get the fan speed
{
  const byte readByte = _EEPROMR;
  StoveSerial.write(readByte);
  nonBlockingDelay(1);
  StoveSerial.write(fanSetAddr);
  digitalWrite(ENABLE_RX, LOW);
  nonBlockingDelay(80);
  checkStoveReply();
}

void getTempSet() // Get the thermostat setting
{
  const byte readByte = _EEPROMR;
  StoveSerial.write(readByte);
  nonBlockingDelay(1);
  StoveSerial.write(tempSetAddr);
  digitalWrite(ENABLE_RX, LOW);
  nonBlockingDelay(80);
  checkStoveReply();
}

void getStates() // Calls all the get…() functions
{
  getStoveState();
  nonBlockingDelay(100);
  getAmbTemp();
  nonBlockingDelay(100);
  getTempSet();
  nonBlockingDelay(100);
  getFumeTemp();
  nonBlockingDelay(100);
  getFlamePower();
  nonBlockingDelay(100);
  getFanSpeed();
  nonBlockingDelay(100);
  client.publish(connection_topic, "Connected");
}

void setup() {
  digitalWrite(ENABLE_RX, HIGH);
  digitalWrite(PIN_RESET, HIGH);
  Serial.begin(115200);
  StoveSerial.begin(1200, SERIAL_MODE, RX_PIN, TX_PIN, false, 256);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  pinMode(ENABLE_RX, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();
  ArduinoOTA.handle();

  unsigned long updatePeriod = fastUpdate ? FAST_UPDATE_PERIOD : UPDATE_PERIOD;
  if ((millis() - previousMillis > updatePeriod) || boot) {
    boot = false;
    previousMillis = millis();
    if (!sem) {
      setSemaphore(true);
      getStates();
      setSemaphore(false);
    }
    client.publish(connection_topic, "Connected");
    if (fastUpdate) loopCounter++;
    if (loopCounter > FAST_LOOP_CYCLES) { fastUpdate = false; loopCounter = 0; }
    resetCounter++;
  }

  if (resetCounter > RESET_CYCLES) {
    digitalWrite(PIN_RESET, LOW);
    // ESP.restart(); // cleaner, but still to be debugged
  }
}

