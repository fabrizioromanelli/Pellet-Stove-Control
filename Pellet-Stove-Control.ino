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
#define UPDATE_PERIOD  10000
#define FAST_UPDATE_PERIOD 600000
#define RESTART_PERIOD 20000
#else
#define UPDATE_PERIOD 3600000     // 1h (in ms)
#define FAST_UPDATE_PERIOD 300000 // 5m (in ms)
#define RESTART_PERIOD 172800000  // 2d (in ms)
#endif

#define FAST_LOOP_CYCLES 10 // Number of cycles for the fast update period.

SoftwareSerial StoveSerial;
#define SERIAL_MODE SWSERIAL_8N2 // 8 data bits, parity none, 2 stop bits
#define RX_PIN D3
#define TX_PIN D4
#define ENABLE_RX D2

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wm;

static bool sem = false;
bool firstLoop  = true;
bool fastUpdate = false;
static unsigned int loopCounter = 0;

unsigned long previousMillis, previousRestartMillis;

#define connection_topic mqtt_topic "/connection_state"
#define state_topic mqtt_topic "/state"
#define power_topic mqtt_topic "/power_state"
#define ta_topic mqtt_topic "/ta_state"
#define tf_topic mqtt_topic "/tf_state"
#define p_topic mqtt_topic "/p_state"
#define f_topic mqtt_topic "/f_state"
#define ts_topic mqtt_topic "/ts_state"
#define cmd mqtt_topic "/cmd"

#define device_information "{\"manufacturer\": \"Fabrizio Romanelli\",\"identifiers\": [\"79F4C268-3947-52FA-A97F-6753F31DC625\"],\"model\": \"Pellet Stove Control\",\"name\": \"Micronova Pellet Stove Control\",\"sw_version\": \"1.0.0.0\"}"

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

// Commands
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
char stoveRxData[2];

void setup_wifi()
{
  ArduinoOTA.setHostname(mqtt_topic);
  ArduinoOTA.setPassword("micronova");
  ArduinoOTA.begin();
  WiFi.mode(WIFI_STA);
  wm.setConnectTimeout(30);
  wm.autoConnect(mqtt_topic);
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESPClient-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass))
    {
      client.setBufferSize(1024);
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

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
        delay(1);
      }
      client.publish(power_topic, "ON", true);
      delay(2000);
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
        delay(1);
      }

      client.publish(power_topic, "OFF", true);
      delay(2000);
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
          delay(1);
        }
        delay(2000);
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
          delay(1);
        }
        delay(2000);
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
          delay(1);
        }
        delay(2000);
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
      delay(400);
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
        delay(1);
      }

      client.publish(power_topic, "OFF", true);
      delay(2000);
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
  while (StoveSerial.available())
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
            delay(1000);
            client.publish(power_topic, "OFF", true);
            break;
          case 1:
            client.publish(state_topic, "Avvio", true);
            delay(1000);
            client.publish(power_topic, "ON", true);
            break;
          case 2:
            client.publish(state_topic, "Caricamento pellet", true);
            delay(1000);
            client.publish(power_topic, "ON", true);
            break;
          case 3:
            client.publish(state_topic, "Accensione", true);
            delay(1000);
            client.publish(power_topic, "ON", true);
            break;
          case 4:
            client.publish(state_topic, "Accesa", true);
            delay(1000);
            client.publish(power_topic, "ON", true);
            break;
          case 5:
            client.publish(state_topic, "Pulizia braciere", true);
            break;
          case 6:
            client.publish(state_topic, "Pulizia finale", true);
            delay(1000);
            client.publish(power_topic, "OFF", true);
            break;
          case 7:
            client.publish(state_topic, "Standby", true);
            delay(1000);
            client.publish(power_topic, "OFF", true);
            break;
          case 8:
            client.publish(state_topic, "Pellet mancante", true);
            break;
          case 9:
            client.publish(state_topic, "Mancata accensione", true);
            delay(1000);
            client.publish(power_topic, "OFF", true);
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
        client.publish(ta_topic, String(ambTemp).c_str(), true);
#ifdef DEBUG
        Serial.print("T. amb. ");
        Serial.println(ambTemp);
#endif
        break;
      case fumesTempAddr:
        fumesTemp = val;
        client.publish(tf_topic, String(fumesTemp).c_str(), true);
#ifdef DEBUG
        Serial.printf("T. fumes %d\n", fumesTemp);
#endif
        break;
      // EEPROM Cases
      case tempSetAddr + _EEPROMR:
        tempSet = val;
        client.publish(ts_topic, String(tempSet).c_str(), true);
#ifdef DEBUG
        Serial.printf("T. set %d\n", tempSet);
#endif
        break;
      case powerSetAddr + _EEPROMR:
        flamePower = val;
        client.publish(p_topic, String(flamePower).c_str(), true);
#ifdef DEBUG
        Serial.printf("Flame Power %d\n", flamePower);
#endif
        break;
      case fanSetAddr + _EEPROMR:
        fanSpeed = val;
        client.publish(f_topic, String(fanSpeed).c_str(), true);
#ifdef DEBUG
        Serial.printf("Fan Speed %d\n", fanSpeed);
#endif
        break;
    }
  }
}

void getStoveState()
{
  const byte readByte = _RAMR;
#ifdef DEBUG
  Serial.printf("DBGXXX1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGXXX2");
#endif
  delay(1);
  StoveSerial.write(stoveStateAddr);
#ifdef DEBUG
  Serial.printf("DBGXXX3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGXXX4");
#endif
  delay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGXXX5");
#endif
}

void getAmbTemp()
{
  const byte readByte = _RAMR;
#ifdef DEBUG
  Serial.printf("DBGYYY1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGYYY2");
#endif
  delay(1);
  StoveSerial.write(ambTempAddr);
#ifdef DEBUG
  Serial.printf("DBGYYY3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGYYY4");
#endif
  delay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGYYY5");
#endif
}

void getFumeTemp()
{
  const byte readByte = _RAMR;
#ifdef DEBUG
  Serial.printf("DBGZZZ1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGZZZ2");
#endif
  delay(1);
  StoveSerial.write(fumesTempAddr);
#ifdef DEBUG
  Serial.printf("DBGZZZ3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGZZZ4");
#endif
  delay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGZZZ5");
#endif
}

void getFlamePower()
{
  const byte readByte = _EEPROMR;
#ifdef DEBUG
  Serial.printf("DBGKKK1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGKKK2");
#endif
  delay(1);
  StoveSerial.write(powerSetAddr);
#ifdef DEBUG
  Serial.printf("DBGKKK3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGKKK4");
#endif
  delay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGKKK5");
#endif
}

void getFanSpeed()
{
  const byte readByte = _EEPROMR;
#ifdef DEBUG
  Serial.printf("DBGLLL1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGLLL2");
#endif
  delay(1);
  StoveSerial.write(fanSetAddr);
#ifdef DEBUG
  Serial.printf("DBGLLL3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGLLL4");
#endif
  delay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGLLL5");
#endif
}

void getTempSet()
{
  const byte readByte = _EEPROMR;
#ifdef DEBUG
  Serial.printf("DBGOOO1");
#endif
  StoveSerial.write(readByte);
#ifdef DEBUG
  Serial.printf("DBGOOO2");
#endif
  delay(1);
  StoveSerial.write(tempSetAddr);
#ifdef DEBUG
  Serial.printf("DBGOOO3");
#endif
  digitalWrite(ENABLE_RX, LOW);
#ifdef DEBUG
  Serial.printf("DBGOOO4");
#endif
  delay(80);
  checkStoveReply();
#ifdef DEBUG
  Serial.printf("DBGOOO5");
#endif
}

void getStates()
{
  getStoveState();
  delay(100);
  getAmbTemp();
  delay(100);
  getTempSet();
  delay(100);
  getFumeTemp();
  delay(100);
  getFlamePower();
  delay(100);
  getFanSpeed();
  delay(100);
  client.publish(connection_topic, "Connected");
}

#ifdef DEBUG
void getDBG(byte input)
{
INIT:
  //  const byte readByte = _EEPROMR;
  const byte readByte = _RAMR;
  StoveSerial.write(readByte);
  delay(1);
  StoveSerial.write(input);
  digitalWrite(ENABLE_RX, LOW);
  delay(80);
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
  pinMode(ENABLE_RX, OUTPUT);
  digitalWrite(ENABLE_RX, HIGH);
  Serial.begin(115200);
  StoveSerial.begin(1200, SERIAL_MODE, RX_PIN, TX_PIN, false, 256);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.subscribe(cmd);
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
  unsigned long currentMillis = millis();
  if (previousMillis > currentMillis)
  {
    previousMillis = 0;
  }

  unsigned long updatePeriod = 0;

  if (fastUpdate)
    updatePeriod = FAST_UPDATE_PERIOD;
  else
    updatePeriod = UPDATE_PERIOD;

  if (currentMillis - previousMillis >= updatePeriod || firstLoop)
  {
    previousMillis = currentMillis;
    if (!sem)
    {
      sem = true;
#ifdef DEBUG
      Serial.println();
      Serial.printf("Loop Start");
      Serial.println();
#endif
      getStates();
#ifdef DEBUG
      Serial.println();
      Serial.printf("Loop Stop");
      Serial.println();
#endif
      sem = false;
      firstLoop = false;
    }

    client.publish(connection_topic, "Connected");

    if (fastUpdate)
      loopCounter++;

    if (loopCounter > FAST_LOOP_CYCLES)
    {
      fastUpdate = false;
      loopCounter = 0;
    }
  }

  if (previousRestartMillis > currentMillis)
    previousRestartMillis = 0;

  // Known bug! We need to restart ESP after RESTART_PERIOD as
  // something in stack or heap crashes if it loops for too long.
  if (currentMillis - previousRestartMillis >= RESTART_PERIOD)
    ESP.restart();
}
