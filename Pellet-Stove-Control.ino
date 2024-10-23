#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

//#define DEBUG

#define mqtt_server ""
#define mqtt_port 1883
#define mqtt_topic ""
#define mqtt_user ""
#define mqtt_pass ""
#ifdef DEBUG
unsigned long UPDATE_PERIOD = 10000;
unsigned long FAST_UPDATE_PERIOD = 600000;
#else
unsigned long UPDATE_PERIOD = 1800000;    // 30m (in ms)
unsigned long FAST_UPDATE_PERIOD = 60000; // 1m (in ms)
#endif

#define FAST_LOOP_CYCLES 30 // FAST_LOOP_CYCLES * FAST_UPDATE_PERIOD = total number of ms update
#define RESET_CYCLES 60

SoftwareSerial StoveSerial;
#define SERIAL_MODE SWSERIAL_8N2 // 8 data bits, parity none, 2 stop bits
#define RX_PIN D3
#define TX_PIN D4
#define ENABLE_RX D2
#define PIN_RESET D0

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wm;

static bool sem = false;
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

//0 - OFF, 1 - Starting, 2 - Pellet loading, 3 - Ignition, 4 - Work, 5 - Brazier cleaning, 6 - Final cleaning, 7 - Standby, 8 - Pellet missing alarm, 9 - Ignition failure alarm, 10 - Alarms (to be investigated)

// RAM defines
#define _RAMR           0x00
#define _RAMW           0x80
#define stoveStateAddr  0x21
#define ambTempAddr     0x01
#define fumesTempAddr   0x3E
#define stovePOnOffAddr 0x58

// EEPROM defines
#define _EEPROMR        0x20
#define _EEPROMW        0xA0
#define powerSetAddr    0x89
#define fanSetAddr      0x8A
#define tempSetAddr     0x8B

// Checksum: Code+Address+Value (hex)
const char stoveOnOff[4] = {_RAMW, stovePOnOffAddr, 0x5A, (char)(_RAMW + stovePOnOffAddr + 0x5A)};
const char stoveOff[4]   = {_RAMW, stoveStateAddr, 0x00, (char)(_RAMW + stoveStateAddr + 0x00)};

const char stovePWA[4]   = {_EEPROMW, powerSetAddr, 0x00, (char)(_EEPROMW + powerSetAddr + 0x00)};
const char stovePW1[4]   = {_EEPROMW, powerSetAddr, 0x01, (char)(_EEPROMW + powerSetAddr + 0x01)};
const char stovePW2[4]   = {_EEPROMW, powerSetAddr, 0x02, (char)(_EEPROMW + powerSetAddr + 0x02)};
const char stovePW3[4]   = {_EEPROMW, powerSetAddr, 0x03, (char)(_EEPROMW + powerSetAddr + 0x03)};
const char stovePW4[4]   = {_EEPROMW, powerSetAddr, 0x04, (char)(_EEPROMW + powerSetAddr + 0x04)};
const char stovePW5[4]   = {_EEPROMW, powerSetAddr, 0x05, (char)(_EEPROMW + powerSetAddr + 0x05)};

const char stoveFNA[4]   = {_EEPROMW, fanSetAddr, 0x00, (char)(_EEPROMW + fanSetAddr + 0x00)};
const char stoveFN1[4]   = {_EEPROMW, fanSetAddr, 0x01, (char)(_EEPROMW + fanSetAddr + 0x01)};
const char stoveFN2[4]   = {_EEPROMW, fanSetAddr, 0x02, (char)(_EEPROMW + fanSetAddr + 0x02)};
const char stoveFN3[4]   = {_EEPROMW, fanSetAddr, 0x03, (char)(_EEPROMW + fanSetAddr + 0x03)};
const char stoveFN4[4]   = {_EEPROMW, fanSetAddr, 0x04, (char)(_EEPROMW + fanSetAddr + 0x04)};
const char stoveFN5[4]   = {_EEPROMW, fanSetAddr, 0x05, (char)(_EEPROMW + fanSetAddr + 0x05)};

uint8_t stoveState, tempSet, fumesTemp, flamePower, fanSpeed;
float ambTemp;
char stoveRxData[2]; // When the heater is sending data, it sends two bytes: a checksum and the value

inline void safeDelay(unsigned int delay)
{
  unsigned long currentMillis = millis();
  while (millis() - currentMillis < delay) {}
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

void reconnect() // Connect to MQTT server
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    char clientId[15];
    int randomNum = random(0xffff);
    snprintf(clientId, sizeof(clientId), "ESPClient-%X", randomNum);
    if (client.connect(clientId, mqtt_user, mqtt_pass))
    {
      client.setBufferSize(1024);
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      safeDelay(5000);
    }
  }
}

// This is the callback from a topic arrived via MQTT
void callback(char *topic, byte *payload, unsigned int length)
{
#ifdef DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++)
    Serial.print((char)payload[i]);
  Serial.println();
#endif

  if ((char)payload[0] == 'O' && (char)payload[1] == 'N') // Switch ON payload
  {
    if (!sem)
    {
      sem = true;
      for (int i = 0; i < 4; i++)
      {
        StoveSerial.write(stoveOnOff[i]);
        safeDelay(1);
      }
      client.publish(onoff_topic, "ON", true);
      safeDelay(2000);
      sem = false;
    }

    if (!sem)
    {
      sem = true;
      getStates();
      sem = false;
    }

    fastUpdate = true;
  }
  else if ((char)payload[0] == 'O' && (char)payload[1] == 'F') // Switch OFF payload
  {
    if (!sem)
    {
      sem = true;
      for (int i = 0; i < 4; i++)
      {
        StoveSerial.write(stoveOff[i]);
        safeDelay(1);
      }

      client.publish(onoff_topic, "OFF", true);
      safeDelay(2000);
      sem = false;
    }

    if (!sem)
    {
      sem = true;
      getStates();
      sem = false;
    }

    fastUpdate = true;
  }
  else if ((char)payload[0] == 'P') // Power payload
  {
    if (length == 2)
    {
      const char *stovePW = NULL;
      if ((char)payload[1] == '0')
        stovePW = stovePWA;
      else if ((char)payload[1] == '1')
        stovePW = stovePW1;
      else if ((char)payload[1] == '2')
        stovePW = stovePW2;
      else if ((char)payload[1] == '3')
        stovePW = stovePW3;
      else if ((char)payload[1] == '4')
        stovePW = stovePW4;
      else if ((char)payload[1] == '5')
        stovePW = stovePW5;

      if (!sem)
      {
        sem = true;
        for (int i = 0; i < 4; i++)
        {
          StoveSerial.write(stovePW[i]);
          safeDelay(1);
        }
        safeDelay(2000);
        sem = false;
      }

      if (!sem)
      {
        sem = true;
        getStates();
        sem = false;
      }
    }
  }
  else if ((char)payload[0] == 'F') // Fan speed payload
  {
    if (length == 2)
    {
      const char *stoveFN = NULL;
      if ((char)payload[1] == '0')
        stoveFN = stoveFNA;
      else if ((char)payload[1] == '1')
        stoveFN = stoveFN1;
      else if ((char)payload[1] == '2')
        stoveFN = stoveFN2;
      else if ((char)payload[1] == '3')
        stoveFN = stoveFN3;
      else if ((char)payload[1] == '4')
        stoveFN = stoveFN4;
      else if ((char)payload[1] == '5')
        stoveFN = stoveFN5;

      if (!sem)
      {
        sem = true;
        for (int i = 0; i < 4; i++)
        {
          StoveSerial.write(stoveFN[i]);
          safeDelay(1);
        }
        safeDelay(2000);
        sem = false;
      }

      if (!sem)
      {
        sem = true;
        getStates();
        sem = false;
      }
    }
  }
  else if ((char)payload[0] == 'T') // Temperature target payload
  {
    if (length == 3)
    {
      char t[3] = {(char)payload[1], (char)payload[2]};
      t[2] = '\0';
      char tempValue = (char)strtol(t, NULL, 10);
      const char temperatureSet[4] = {_EEPROMW, tempSetAddr, tempValue, (char)(_EEPROMW + tempSetAddr + tempValue)};

      if (!sem)
      {
        sem = true;
        for (int i = 0; i < 4; i++)
        {
          StoveSerial.write(temperatureSet[i]);
          safeDelay(1);
        }
        safeDelay(2000);
        sem = false;
      }

      if (!sem)
      {
        sem = true;
        getStates();
        sem = false;
      }
    }
  }
  else if ((char)payload[0] == 'D') // Dump all the registers from 0x00 to 0xFF payload
  {
#ifdef DEBUG
    for (char t = 0x00; t < 0xFF; t++)
    {
      Serial.printf("%01X -> ", t);
      getDBG(t);
      Serial.println();
      safeDelay(400);
    }
#endif
  }
  else if ((char)payload[0] == 'R') // Read one register from payload
  {
#ifdef DEBUG
    char t[3] = {(char)payload[1], (char)payload[2]};
    t[2] = '\0';
    char num = (char)strtol(t, NULL, 16);
    getDBG(num);
#endif
  }
  else if ((char)payload[0] == 'E') // Reset error (it's the same as sending an OFF request)
  {
    if (!sem)
    {
      sem = true;
      for (int i = 0; i < 4; i++)
      {
        StoveSerial.write(stoveOff[i]);
        safeDelay(1);
      }

      client.publish(onoff_topic, "OFF", true);
      safeDelay(2000);
      sem = false;
    }

    if (!sem)
    {
      sem = true;
      getStates();
      sem = false;
    }
  }
}

void checkStoveReply()
{
  uint8_t rxCount = 0;
  stoveRxData[0] = 0x00;
  stoveRxData[1] = 0x00;
#ifdef DEBUG
  Serial.printf("DBG01");
#endif
  while (StoveSerial.available()) // It has to be exactly 2 bytes, otherwise it's an error
  {
    stoveRxData[rxCount] = StoveSerial.read();
    rxCount++;
  }
#ifdef DEBUG
  Serial.printf("DBG02");
#endif
  digitalWrite(ENABLE_RX, HIGH);
#ifdef DEBUG
  Serial.printf("DBG03");
#endif
  if (rxCount == 2)
  {
    byte val = stoveRxData[1];
    byte checksum = stoveRxData[0];
    byte param = checksum - val;
#ifdef DEBUG
    Serial.printf("Param=%01x value=%01x ", param, val);
#endif
    switch (param)
    {
      // RAM Cases
      case stoveStateAddr:
        stoveState = val;
        switch (stoveState)
        {
          case 0:
            client.publish(state_topic, "Spenta", true);
            safeDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 1:
            client.publish(state_topic, "Avvio", true);
            safeDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 2:
            client.publish(state_topic, "Caricamento pellet", true);
            safeDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 3:
            client.publish(state_topic, "Accensione", true);
            safeDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 4:
            client.publish(state_topic, "Accesa", true);
            safeDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 5:
            client.publish(state_topic, "Pulizia braciere", true);
            break;
          case 6:
            client.publish(state_topic, "Pulizia finale", true);
            safeDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 7:
            client.publish(state_topic, "Standby", true);
            safeDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 8:
            client.publish(state_topic, "Pellet mancante", true);
            break;
          case 9:
            client.publish(state_topic, "Mancata accensione", true);
            safeDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 10:
            client.publish(state_topic, "Allarme", true);
            break;
        }
#ifdef DEBUG
        Serial.printf("Stove %s\n", stoveState ? "ON" : "OFF");
#endif
        break;
      case ambTempAddr:
        ambTemp = (float)val / 2;
        char ambTempStr[10];
        dtostrf(ambTemp, 4, 2, ambTempStr);
        client.publish(ambtemp_topic, ambTempStr, true);
#ifdef DEBUG
        Serial.print("T. amb. ");
        Serial.println(ambTemp);
#endif
        break;
      case fumesTempAddr:
        fumesTemp = val;
        char fumesTempStr[10];
        sprintf(fumesTempStr, "%d", fumesTemp);
        client.publish(fumetemp_topic, fumesTempStr, true);
#ifdef DEBUG
        Serial.printf("T. fumes %d\n", fumesTemp);
#endif
        break;
      // EEPROM Cases
      case tempSetAddr + _EEPROMR:
        tempSet = val;
        char tempSetStr[10];
        sprintf(tempSetStr, "%d", tempSet);
        client.publish(tempset_topic, tempSetStr, true);
#ifdef DEBUG
        Serial.printf("T. set %d\n", tempSet);
#endif
        break;
      case powerSetAddr + _EEPROMR:
        flamePower = val;
        char flamePowerStr[10];
        sprintf(flamePowerStr, "%d", flamePower);
        client.publish(flame_topic, flamePowerStr, true);
#ifdef DEBUG
        Serial.printf("Flame Power %d\n", flamePower);
#endif
        break;
      case fanSetAddr + _EEPROMR:
        fanSpeed = val;
        char fanSpeedStr[10];
        sprintf(fanSpeedStr, "%d", fanSpeed);
        client.publish(fan_topic, fanSpeedStr, true);
#ifdef DEBUG
        Serial.printf("Fan Speed %d\n", fanSpeed);
#endif
        break;
    }
  }
}

void getStoveState() // Get detailed stove state
{
  const byte readByte = _RAMR;
#ifdef DEBUG
  Serial.printf("DBGXXX1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGXXX2");
#endif
  safeDelay(1);
  StoveSerial.write(stoveStateAddr);
#ifdef DEBUG
  Serial.printf("DBGXXX3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGXXX4");
#endif
  safeDelay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGXXX5");
#endif
}

void getAmbTemp() // Get room temperature
{
  const byte readByte = _RAMR;
#ifdef DEBUG
  Serial.printf("DBGYYY1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGYYY2");
#endif
  safeDelay(1);
  StoveSerial.write(ambTempAddr);
#ifdef DEBUG
  Serial.printf("DBGYYY3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGYYY4");
#endif
  safeDelay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGYYY5");
#endif
}

void getFumeTemp() // Get flue gas temperature
{
  const byte readByte = _RAMR;
#ifdef DEBUG
  Serial.printf("DBGZZZ1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGZZZ2");
#endif
  safeDelay(1);
  StoveSerial.write(fumesTempAddr);
#ifdef DEBUG
  Serial.printf("DBGZZZ3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGZZZ4");
#endif
  safeDelay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGZZZ5");
#endif
}

void getFlamePower() // Get the flame power
{
  const byte readByte = _EEPROMR;
#ifdef DEBUG
  Serial.printf("DBGKKK1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGKKK2");
#endif
  safeDelay(1);
  StoveSerial.write(powerSetAddr);
#ifdef DEBUG
  Serial.printf("DBGKKK3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGKKK4");
#endif
  safeDelay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGKKK5");
#endif
}

void getFanSpeed() // Get the fan speed
{
  const byte readByte = _EEPROMR;
#ifdef DEBUG
  Serial.printf("DBGLLL1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGLLL2");
#endif
  safeDelay(1);
  StoveSerial.write(fanSetAddr);
#ifdef DEBUG
  Serial.printf("DBGLLL3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGLLL4");
#endif
  safeDelay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGLLL5");
#endif
}

void getTempSet() // Get the thermostat setting
{
  const byte readByte = _EEPROMR;
#ifdef DEBUG
  Serial.printf("DBGOOO1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGOOO2");
#endif
  safeDelay(1);
  StoveSerial.write(tempSetAddr);
#ifdef DEBUG
  Serial.printf("DBGOOO3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGOOO4");
#endif
  safeDelay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGOOO5");
#endif
}

void getStates() //Calls all the get…() functions
{
  getStoveState();
  safeDelay(100);
  getAmbTemp();
  safeDelay(100);
  getTempSet();
  safeDelay(100);
  getFumeTemp();
  safeDelay(100);
  getFlamePower();
  safeDelay(100);
  getFanSpeed();
  safeDelay(100);
  client.publish(connection_topic, "Connected");
}

#ifdef DEBUG
void getDBG(byte input)
{
INIT:
  //  const byte readByte = _EEPROMR;
  const byte readByte = _RAMR;
  StoveSerial.write(readByte);
  safeDelay(1);
  StoveSerial.write(input);
  digitalWrite(ENABLE_RX, LOW);
  safeDelay(80);
  uint8_t rxCount = 0;
  stoveRxData[0] = 0x00;
  stoveRxData[1] = 0x00;
  while (StoveSerial.available())
  {
    stoveRxData[rxCount] = StoveSerial.read();
    rxCount++;
  }
  digitalWrite(ENABLE_RX, HIGH);
  if (rxCount == 2)
  {
    byte val = stoveRxData[1];
    byte checksum = stoveRxData[0];
    byte param = checksum - val;
    Serial.printf("Param=%01x value=%01x ", param, val);
  } else {
    goto INIT;
    Serial.printf("rxCount = %d", rxCount);
  }
}
#endif

void setup()
{
  digitalWrite(ENABLE_RX, HIGH);
  digitalWrite(PIN_RESET, HIGH);
  Serial.begin(115200);
  StoveSerial.begin(1200, SERIAL_MODE, RX_PIN, TX_PIN, false, 256);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.subscribe(cmd);
  pinMode(ENABLE_RX, OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
    client.subscribe(cmd);
  }
  client.loop();
  ArduinoOTA.handle();
  int updatePeriod = 0;

  if (fastUpdate)
    updatePeriod = FAST_UPDATE_PERIOD;
  else
    updatePeriod = UPDATE_PERIOD;

  if (((unsigned long)(millis() - previousMillis) > updatePeriod) || boot)
  {
    boot = false;
    previousMillis = millis();
    if (!sem)
    {
      sem = true;
#ifdef DEBUG
      Serial.println();
      Serial.printf("Loop Start");
      Serial.println();
      Serial.println(ESP.getFreeHeap());
#endif
      getStates();
#ifdef DEBUG
      Serial.println();
      Serial.printf("Loop Stop");
      Serial.println();
      Serial.println(ESP.getFreeHeap());
#endif
      sem = false;
    }

    client.publish(connection_topic, "Connected");

    if (fastUpdate)
      loopCounter++;

    if (loopCounter > FAST_LOOP_CYCLES)
    {
      fastUpdate = false;
      loopCounter = 0;
    }
    resetCounter++;
  }

  if (resetCounter > RESET_CYCLES)
  {
#ifdef DEBUG
    Serial.printf("Reset request");
#endif
    digitalWrite(PIN_RESET, LOW);
  }
}