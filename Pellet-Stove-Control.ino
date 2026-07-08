#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <LittleFS.h>

#define AP_NAME "PelletStove-Setup"    // Access point name shown on first boot / config reset
#define CONFIG_FILE "/mqtt_config.txt" // Where MQTT settings are persisted on LittleFS

char mqtt_server[41] = "";
char mqtt_port[7]    = "1883";
char mqtt_topic[41]  = "";
char mqtt_user[21]   = "";
char mqtt_pass[21]   = "";

bool shouldSaveConfig = false; // Set by WiFiManager when the config portal is submitted

unsigned long UPDATE_PERIOD = 1800000; // 30m (in ms)
unsigned long FAST_UPDATE_PERIOD = 60000; // 1m (in ms)

#define FAST_LOOP_CYCLES 30

SoftwareSerial StoveSerial;
#define SERIAL_MODE SWSERIAL_8N2
#define RX_PIN D3
#define TX_PIN D4
#define ENABLE_RX D2

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wm;

static volatile bool sem = false;
bool fastUpdate = false;
bool boot = true;
static unsigned long loopCounter = 0;
unsigned long previousMillis = 0;

char connection_topic[52];
char state_topic[52];
char onoff_topic[52];
char ambtemp_topic[52];
char fumetemp_topic[52];
char flame_topic[52];
char fan_topic[52];
char tempset_topic[52];
char cmd_topic[52];

// mqtt_topic is only known at runtime now, so the full topics are built here instead of via macros
void buildTopics() {
  snprintf(connection_topic, sizeof(connection_topic), "%s/connection", mqtt_topic);
  snprintf(state_topic,      sizeof(state_topic),      "%s/state",      mqtt_topic);
  snprintf(onoff_topic,      sizeof(onoff_topic),      "%s/onoff",      mqtt_topic);
  snprintf(ambtemp_topic,    sizeof(ambtemp_topic),    "%s/ambtemp",    mqtt_topic);
  snprintf(fumetemp_topic,   sizeof(fumetemp_topic),   "%s/fumetemp",   mqtt_topic);
  snprintf(flame_topic,      sizeof(flame_topic),      "%s/power",      mqtt_topic);
  snprintf(fan_topic,        sizeof(fan_topic),        "%s/fan",        mqtt_topic);
  snprintf(tempset_topic,    sizeof(tempset_topic),    "%s/tempset",    mqtt_topic);
  snprintf(cmd_topic,        sizeof(cmd_topic),        "%s/cmd",        mqtt_topic);
}

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

// Loads previously saved MQTT settings from LittleFS, if present
bool loadConfig() {
  if (!LittleFS.begin()) return false;
  if (!LittleFS.exists(CONFIG_FILE)) return false;

  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return false;

  auto readLine = [&](char* buf, size_t len) {
    String s = f.readStringUntil('\n');
    s.trim();
    strncpy(buf, s.c_str(), len - 1);
    buf[len - 1] = '\0';
  };

  readLine(mqtt_server, sizeof(mqtt_server));
  readLine(mqtt_port, sizeof(mqtt_port));
  readLine(mqtt_topic, sizeof(mqtt_topic));
  readLine(mqtt_user, sizeof(mqtt_user));
  readLine(mqtt_pass, sizeof(mqtt_pass));
  f.close();

  return strlen(mqtt_topic) > 0;
}

// Persists the current MQTT settings to LittleFS
void saveConfig() {
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return;
  f.println(mqtt_server);
  f.println(mqtt_port);
  f.println(mqtt_topic);
  f.println(mqtt_user);
  f.println(mqtt_pass);
  f.close();
}

void saveConfigCallback() {
  shouldSaveConfig = true;
}

void setup_wifi() // Setup WiFiManager, ask for MQTT settings on first boot and connect to WiFi
{
  WiFiManagerParameter custom_mqtt_server("server", "MQTT server", mqtt_server, sizeof(mqtt_server));
  WiFiManagerParameter custom_mqtt_port("port", "MQTT port", mqtt_port, sizeof(mqtt_port));
  WiFiManagerParameter custom_mqtt_topic("topic", "MQTT base topic", mqtt_topic, sizeof(mqtt_topic));
  WiFiManagerParameter custom_mqtt_user("user", "MQTT user", mqtt_user, sizeof(mqtt_user));
  WiFiManagerParameter custom_mqtt_pass("pass", "MQTT password", mqtt_pass, sizeof(mqtt_pass));

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_topic);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
  wm.setSaveConfigCallback(saveConfigCallback);

  WiFi.mode(WIFI_STA);
  wm.setConnectTimeout(30);
  wm.autoConnect(AP_NAME);

  strncpy(mqtt_server, custom_mqtt_server.getValue(), sizeof(mqtt_server) - 1);
  strncpy(mqtt_port, custom_mqtt_port.getValue(), sizeof(mqtt_port) - 1);
  strncpy(mqtt_topic, custom_mqtt_topic.getValue(), sizeof(mqtt_topic) - 1);
  strncpy(mqtt_user, custom_mqtt_user.getValue(), sizeof(mqtt_user) - 1);
  strncpy(mqtt_pass, custom_mqtt_pass.getValue(), sizeof(mqtt_pass) - 1);

  if (shouldSaveConfig) {
    saveConfig();
    shouldSaveConfig = false;
  }
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
      client.subscribe(cmd_topic);
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
  nonBlockingDelay(100);z
  client.publish(connection_topic, "Connected");
}

void setup() {
  digitalWrite(ENABLE_RX, HIGH);
  Serial.begin(115200);
  StoveSerial.begin(1200, SERIAL_MODE, RX_PIN, TX_PIN, false, 256);

  loadConfig();   // Pre-fill defaults from LittleFS, if a config was already saved
  setup_wifi();   // Connects to WiFi; shows the config portal (with MQTT fields) if needed
  buildTopics();  // mqtt_topic is only known now, so build the full topic strings

  ArduinoOTA.setHostname(mqtt_topic[0] ? mqtt_topic : AP_NAME);
  ArduinoOTA.setPassword("micronova");
  ArduinoOTA.begin();

  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setCallback(callback);
  pinMode(ENABLE_RX, OUTPUT);
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
  }
}
