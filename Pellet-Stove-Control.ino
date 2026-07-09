#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <LittleFS.h>

#define AP_NAME "PelletStove-Setup"    // Access point name shown on first boot / config reset
#define CONFIG_FILE "/mqtt_config.txt" // Where MQTT settings are persisted on LittleFS

// MQTT connection settings. Left empty/default here; the real values are either restored from
// LittleFS by loadConfig() or collected via the WiFiManager captive portal in setup_wifi().
char mqtt_server[41] = "";
char mqtt_port[7]    = "1883";
char mqtt_topic[41]  = ""; // Base topic all other topics are built from (see buildTopics())
char mqtt_user[21]   = "";
char mqtt_pass[21]   = "";

bool shouldSaveConfig = false; // Set by WiFiManager when the config portal is submitted

// Polling cadence for getStates(): slow when idle, fast right after a command so HA/automations
// see the stove react quickly, then it automatically falls back to the slow cadence (see loop()).
unsigned long UPDATE_PERIOD = 1800000; // 30m (in ms)
unsigned long FAST_UPDATE_PERIOD = 60000; // 1m (in ms)

#define FAST_LOOP_CYCLES 30 // How many fast-cadence polls to run after a command before reverting to slow

SoftwareSerial StoveSerial; // Talks to the Micronova stove board over its proprietary 1200 baud serial bus
#define SERIAL_MODE SWSERIAL_8N2
#define RX_PIN D3     // = GPIO0 on most ESP8266 boards. This is a boot-strapping pin (must read HIGH at
                       // power-on for the chip to boot from flash instead of entering UART flashing mode).
                       // That sampling happens in the ROM bootloader before setup() ever runs, so it isn't
                       // something this sketch can fix in software - just something to remember if the
                       // level-shifter/wiring to the stove board is ever changed.
#define TX_PIN D4     // = GPIO2 on most ESP8266 boards: same boot-strapping caveat as RX_PIN (must be HIGH at boot).
#define ENABLE_RX D2  // Drives the RS-485/level-shifter direction: HIGH = transmit (receiver disabled),
                       // LOW = receive enabled. Toggled around every read exchange with the stove.

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wm;

// sem guards StoveSerial against being written to from two places at once: the periodic
// getStates() poll (driven by loop()) and an MQTT command arriving via callback(). Because the
// ESP8266 Arduino core is single-threaded, this can only actually happen through client.loop()
// being invoked *while already inside* nonBlockingDelay() (i.e. mid-poll) - in that case the
// incoming command is silently dropped rather than colliding with the in-flight UART exchange.
// noInterrupts()/interrupts() in setSemaphore() just make the flag flip atomic with respect to ISRs.
static volatile bool sem = false;
bool fastUpdate = false;  // True for FAST_LOOP_CYCLES polls after any command, see loop()
bool boot = true;         // Forces an immediate getStates() on the very first loop() iteration
static unsigned long loopCounter = 0;
unsigned long previousMillis = 0;

// Full MQTT topics, built at runtime by buildTopics() once mqtt_topic is known (from LittleFS or
// the config portal). Sizes are set to exactly fit mqtt_topic's max length (40 chars) plus the
// longest suffix used below ("/connection", 11 chars) plus the null terminator.
char connection_topic[52]; // Availability/LWT topic: "Connected" (retained) / "Disconnected" (LWT)
char state_topic[52];      // Human-readable stove state (Italian strings, see checkStoveReply())
char onoff_topic[52];      // "ON"/"OFF" mirror of the stove's power state
char ambtemp_topic[52];    // Room temperature, °C
char fumetemp_topic[52];   // Flue gas temperature, °C
char flame_topic[52];      // Flame/power level 0-5 (topic suffix is "power"; see buildTopics())
char fan_topic[52];        // Fan speed level 0-5
char tempset_topic[52];    // Current thermostat set-point, °C
char cmd_topic[52];        // Subscribed topic: accepts ON/OF/Pn/Fn/Tnn/E commands, see callback()
char node_id[41]; // mqtt_topic sanitized for use inside HA discovery topics/unique_ids and as the
                   // ArduinoOTA/mDNS hostname (those contexts don't allow '/')

// mqtt_topic is only known at runtime now, so the full topics are built here instead of via macros.
// Must run after mqtt_topic has its final value (from loadConfig()/setup_wifi()) and before
// anything that uses these buffers (MQTT connect/subscribe, ArduinoOTA hostname, discovery).
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

  // node_id must never contain '/': it's used as a path segment in HA discovery topics and as
  // the ArduinoOTA/mDNS hostname, both of which would otherwise end up malformed if mqtt_topic
  // itself contains multiple path segments (e.g. "casa/stufa").
  strncpy(node_id, mqtt_topic, sizeof(node_id) - 1);
  node_id[sizeof(node_id) - 1] = '\0';
  for (char *p = node_id; *p; p++) if (*p == '/') *p = '_';
}

// HA "device" object embedded in every discovery payload below, so all entities are grouped
// under one device page in Home Assistant instead of appearing as unrelated entities.
#define device_information "{\"manufacturer\": \"Fabrizio Romanelli\",\"identifiers\": [\"7a396f39-80d2-493b-8e8e-31a70e700bc6\"],\"model\": \"Micronova Controller\",\"name\": \"Micronova Pellet Stove Controller\",\"sw_version\": \"1.0.0.0\"}"

#define DISCOVERY_PREFIX "homeassistant" // Must match the "discovery_prefix" configured in HA's MQTT integration (default)
#define STOVE_MIN_TEMP 10 // Adjust to the actual thermostat range accepted by your stove.
#define STOVE_MAX_TEMP 30 // Used both for the climate entity's min/max_temp and to clamp incoming 'T' commands (see callback()).

// Publishes one HA MQTT Discovery config message. component: "climate"/"sensor"/"binary_sensor"/"button".
// topic[] is sized for DISCOVERY_PREFIX + "/" + component + "/" + node_id (40 chars max) + objectSuffix
// + "/config" - worst case ("binary_sensor" + "_connectivity") is ~88 chars, well under 110.
void publishDiscoveryEntity(const char *component, const char *objectSuffix, const char *payload) {
  char topic[110];
  snprintf(topic, sizeof(topic), "%s/%s/%s%s/config", DISCOVERY_PREFIX, component, node_id, objectSuffix);
  client.publish(topic, payload, true);
  nonBlockingDelay(50); // Give the broker a moment between retained discovery messages
}

// Publishes MQTT Discovery configs for every entity, so HA creates/updates them automatically (no
// manual YAML/Automations needed). Called once per successful MQTT (re)connect from reconnect().
// All entities use the "~" topic-prefix shorthand (set to mqtt_topic) to keep the JSON compact,
// and share the same availability_topic (connection_topic) so they go unavailable together if the
// ESP drops off (see the LWT set in reconnect()'s client.connect() call).
void publishDiscovery() {
  // Sized with margin above the largest payload actually produced (the climate entity, ~1.2KB
  // when mqtt_topic/node_id are near their 40-char maximum) - snprintf truncates silently if this
  // is too small, which would publish malformed JSON that Home Assistant simply can't parse.
  static char payload[1536];

  // Climate: single entity for on/off, target temperature, fan speed (fan_mode) and power level (preset_mode)
  snprintf(payload, sizeof(payload),
    "{"
    "\"~\":\"%s\","
    "\"name\":\"Pellet Stove\","
    "\"unique_id\":\"%s_climate\","
    "\"modes\":[\"off\",\"heat\"],"
    "\"mode_command_topic\":\"~/cmd\","
    "\"mode_command_template\":\"{{ 'ON' if value=='heat' else 'OF' }}\","
    "\"mode_state_topic\":\"~/onoff\","
    "\"mode_state_template\":\"{{ 'heat' if value=='ON' else 'off' }}\","
    "\"temperature_command_topic\":\"~/cmd\","
    "\"temperature_command_template\":\"T{{ '%%02d' %% (value | int) }}\","
    "\"temperature_state_topic\":\"~/tempset\","
    "\"current_temperature_topic\":\"~/ambtemp\","
    "\"min_temp\":%d,"
    "\"max_temp\":%d,"
    "\"temp_step\":1,"
    "\"temperature_unit\":\"C\","
    "\"fan_modes\":[\"0\",\"1\",\"2\",\"3\",\"4\",\"5\"],"
    "\"fan_mode_command_topic\":\"~/cmd\","
    "\"fan_mode_command_template\":\"F{{ value }}\","
    "\"fan_mode_state_topic\":\"~/fan\","
    "\"preset_modes\":[\"1\",\"2\",\"3\",\"4\",\"5\"],"
    "\"preset_mode_command_topic\":\"~/cmd\","
    "\"preset_mode_command_template\":\"P{{ value }}\","
    "\"preset_mode_state_topic\":\"~/power\","
    "\"availability_topic\":\"~/connection\","
    "\"payload_available\":\"Connected\","
    "\"payload_not_available\":\"Disconnected\","
    "\"device\":%s"
    "}",
    mqtt_topic, node_id, STOVE_MIN_TEMP, STOVE_MAX_TEMP, device_information);
  publishDiscoveryEntity("climate", "_climate", payload);

  // Button: emergency stop (raw "E" command)
  snprintf(payload, sizeof(payload),
    "{"
    "\"~\":\"%s\","
    "\"name\":\"Pellet Stove Emergency Stop\","
    "\"unique_id\":\"%s_estop\","
    "\"command_topic\":\"~/cmd\","
    "\"payload_press\":\"E\","
    "\"icon\":\"mdi:alert-octagon\","
    "\"availability_topic\":\"~/connection\","
    "\"payload_available\":\"Connected\","
    "\"payload_not_available\":\"Disconnected\","
    "\"device\":%s"
    "}",
    mqtt_topic, node_id, device_information);
  publishDiscoveryEntity("button", "_estop", payload);

  // Sensor: human-readable stove state
  snprintf(payload, sizeof(payload),
    "{"
    "\"~\":\"%s\","
    "\"name\":\"Pellet Stove State\","
    "\"unique_id\":\"%s_state\","
    "\"state_topic\":\"~/state\","
    "\"icon\":\"mdi:information-outline\","
    "\"availability_topic\":\"~/connection\","
    "\"payload_available\":\"Connected\","
    "\"payload_not_available\":\"Disconnected\","
    "\"device\":%s"
    "}",
    mqtt_topic, node_id, device_information);
  publishDiscoveryEntity("sensor", "_state", payload);

  // Sensor: ambient temperature (also feeds climate's current_temperature, kept separate for history/statistics)
  snprintf(payload, sizeof(payload),
    "{"
    "\"~\":\"%s\","
    "\"name\":\"Pellet Stove Ambient Temperature\","
    "\"unique_id\":\"%s_ambtemp\","
    "\"state_topic\":\"~/ambtemp\","
    "\"device_class\":\"temperature\","
    "\"unit_of_measurement\":\"°C\","
    "\"state_class\":\"measurement\","
    "\"availability_topic\":\"~/connection\","
    "\"payload_available\":\"Connected\","
    "\"payload_not_available\":\"Disconnected\","
    "\"device\":%s"
    "}",
    mqtt_topic, node_id, device_information);
  publishDiscoveryEntity("sensor", "_ambtemp", payload);

  // Sensor: flue gas temperature
  snprintf(payload, sizeof(payload),
    "{"
    "\"~\":\"%s\","
    "\"name\":\"Pellet Stove Fumes Temperature\","
    "\"unique_id\":\"%s_fumetemp\","
    "\"state_topic\":\"~/fumetemp\","
    "\"device_class\":\"temperature\","
    "\"unit_of_measurement\":\"°C\","
    "\"state_class\":\"measurement\","
    "\"availability_topic\":\"~/connection\","
    "\"payload_available\":\"Connected\","
    "\"payload_not_available\":\"Disconnected\","
    "\"device\":%s"
    "}",
    mqtt_topic, node_id, device_information);
  publishDiscoveryEntity("sensor", "_fumetemp", payload);

  // Sensor: raw fan speed level (0-5), for history graphs independent of the climate card
  snprintf(payload, sizeof(payload),
    "{"
    "\"~\":\"%s\","
    "\"name\":\"Pellet Stove Fan Speed\","
    "\"unique_id\":\"%s_fanspeed\","
    "\"state_topic\":\"~/fan\","
    "\"icon\":\"mdi:fan\","
    "\"state_class\":\"measurement\","
    "\"availability_topic\":\"~/connection\","
    "\"payload_available\":\"Connected\","
    "\"payload_not_available\":\"Disconnected\","
    "\"device\":%s"
    "}",
    mqtt_topic, node_id, device_information);
  publishDiscoveryEntity("sensor", "_fanspeed", payload);

  // Sensor: raw power level (0-5), for history graphs independent of the climate card
  snprintf(payload, sizeof(payload),
    "{"
    "\"~\":\"%s\","
    "\"name\":\"Pellet Stove Power Level\","
    "\"unique_id\":\"%s_power\","
    "\"state_topic\":\"~/power\","
    "\"icon\":\"mdi:fire\","
    "\"state_class\":\"measurement\","
    "\"availability_topic\":\"~/connection\","
    "\"payload_available\":\"Connected\","
    "\"payload_not_available\":\"Disconnected\","
    "\"device\":%s"
    "}",
    mqtt_topic, node_id, device_information);
  publishDiscoveryEntity("sensor", "_power", payload);

  // Binary sensor: connectivity (diagnostic), mirrors connection_topic directly
  snprintf(payload, sizeof(payload),
    "{"
    "\"~\":\"%s\","
    "\"name\":\"Pellet Stove Connectivity\","
    "\"unique_id\":\"%s_connectivity\","
    "\"state_topic\":\"~/connection\","
    "\"payload_on\":\"Connected\","
    "\"payload_off\":\"Disconnected\","
    "\"device_class\":\"connectivity\","
    "\"entity_category\":\"diagnostic\","
    "\"device\":%s"
    "}",
    mqtt_topic, node_id, device_information);
  publishDiscoveryEntity("binary_sensor", "_connectivity", payload);
}

// Micronova serial protocol basics:
// - A write frame is 4 bytes: [operation code][register address][value][checksum], where
//   checksum = (code + address + value) truncated to 8 bits (see the (char)(...) casts below).
// - A read frame is 2 bytes: [operation code][register address]; the stove replies with 2 bytes:
//   [checksum][value], where checksum = (code + address + value) truncated to 8 bits, same
//   formula as a write. checkStoveReply() recovers "code + address" as "checksum - value" and
//   switches on that. Since _RAMR is 0, RAM registers match their bare address; since _EEPROMR
//   is 0x20, EEPROM registers are matched as "address + _EEPROMR" (see the switch below) - that
//   offset is what actually distinguishes an EEPROM reply from a RAM one at the same address.
// - Two independent register spaces exist, RAM (live/volatile) and EEPROM (persisted settings),
//   selected by the R/W operation code used.
#define _RAMR 0x00 // Read a RAM register (live values: on/off state, live temperatures)
#define _RAMW 0x80 // Write a RAM register
#define stoveStateAddr 0x21   // RAM: overall stove state machine (0-10, see checkStoveReply())
#define ambTempAddr 0x01      // RAM: room temperature, stored as (°C * 2)
#define fumesTempAddr 0x3E    // RAM: flue gas temperature, °C
#define stovePOnOffAddr 0x58  // RAM: power on/off toggle register (see stoveOnOff below)

#define _EEPROMR 0x20 // Read an EEPROM register (persisted settings)
#define _EEPROMW 0xA0 // Write an EEPROM register
#define powerSetAddr 0x89 // EEPROM: flame power level, 0-5
#define fanSetAddr 0x8A    // EEPROM: fan speed level, 0-5
#define tempSetAddr 0x8B   // EEPROM: thermostat set-point, °C

// Pre-built command frames (checksum computed at compile time). Note the asymmetry between
// turning the stove on and off: powering on pulses a dedicated toggle register (stovePOnOffAddr)
// to kick off the stove's own ignition sequence, while powering off writes the state register
// directly to force it back to idle (state 0) - this mirrors how the stove's own remote does it
// and isn't something this sketch can simplify without risking incorrect stove behavior.
const char stoveOnOff[4] = {_RAMW, stovePOnOffAddr, 0x5A, (char)(_RAMW + stovePOnOffAddr + 0x5A)};
const char stoveOff[4] = {_RAMW, stoveStateAddr, 0x00, (char)(_RAMW + stoveStateAddr + 0x00)};

// One pre-built EEPROM-write frame per power level (index = level, 0-5)
const char stovePW[6][4] = {
{_EEPROMW, powerSetAddr, 0x00, (char)(_EEPROMW + powerSetAddr + 0x00)},
{_EEPROMW, powerSetAddr, 0x01, (char)(_EEPROMW + powerSetAddr + 0x01)},
{_EEPROMW, powerSetAddr, 0x02, (char)(_EEPROMW + powerSetAddr + 0x02)},
{_EEPROMW, powerSetAddr, 0x03, (char)(_EEPROMW + powerSetAddr + 0x03)},
{_EEPROMW, powerSetAddr, 0x04, (char)(_EEPROMW + powerSetAddr + 0x04)},
{_EEPROMW, powerSetAddr, 0x05, (char)(_EEPROMW + powerSetAddr + 0x05)}
};

// One pre-built EEPROM-write frame per fan speed level (index = level, 0-5)
const char stoveFN[6][4] = {
{_EEPROMW, fanSetAddr, 0x00, (char)(_EEPROMW + fanSetAddr + 0x00)},
{_EEPROMW, fanSetAddr, 0x01, (char)(_EEPROMW + fanSetAddr + 0x01)},
{_EEPROMW, fanSetAddr, 0x02, (char)(_EEPROMW + fanSetAddr + 0x02)},
{_EEPROMW, fanSetAddr, 0x03, (char)(_EEPROMW + fanSetAddr + 0x03)},
{_EEPROMW, fanSetAddr, 0x04, (char)(_EEPROMW + fanSetAddr + 0x04)},
{_EEPROMW, fanSetAddr, 0x05, (char)(_EEPROMW + fanSetAddr + 0x05)}
};

// Last values read from the stove, cached so checkStoveReply() has somewhere to store each
// register as it comes in; also mirrors what was last published to the corresponding MQTT topic.
uint8_t stoveState, tempSet, fumesTemp, flamePower, fanSpeed;
float ambTemp;
char stoveRxData[2];  // When the stove is sending data, it sends two bytes: a checksum and the value

// Blocks for `ms` milliseconds without blocking MQTT/OTA: pumps client.loop() (so incoming MQTT
// packets and the keep-alive are still processed) and ArduinoOTA.handle() (so an OTA update can't
// be starved by a long wait) on every iteration. Used everywhere instead of delay().
void nonBlockingDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    client.loop();
    ArduinoOTA.handle();
    yield();
  }
}

// Atomically flips the sem flag (see its declaration above for what it protects against).
void setSemaphore(bool value) {
  noInterrupts();
  sem = value;
  interrupts();
}

// Writes a 4-byte command frame to the stove, one byte at a time with a 1ms gap (the stove's UART
// is slow/basic and expects bytes paced rather than bursted), then waits 50ms for it to process
// before releasing the bus. If sem is already held (a getStates() poll is mid-flight), the command
// is silently skipped rather than queued - see the sem comment above for why that's the trade-off.
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
  // Pre-filled with whatever loadConfig() restored from LittleFS, so returning users just see
  // their previous values instead of blank fields.
  WiFiManagerParameter custom_mqtt_server("server", "MQTT server", mqtt_server, sizeof(mqtt_server));
  WiFiManagerParameter custom_mqtt_port("port", "MQTT port", mqtt_port, sizeof(mqtt_port));
  WiFiManagerParameter custom_mqtt_topic("topic", "MQTT base topic", mqtt_topic, sizeof(mqtt_topic));
  WiFiManagerParameter custom_mqtt_user("user", "MQTT user", mqtt_user, sizeof(mqtt_user));
  // type="password" masks the field in the captive portal - without it WiFiManager renders a
  // plain text input and the MQTT password would be visible on screen while typing/reviewing it.
  WiFiManagerParameter custom_mqtt_pass("pass", "MQTT password", mqtt_pass, sizeof(mqtt_pass), "type=\"password\"");

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_topic);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
  wm.setSaveConfigCallback(saveConfigCallback); // Just flags shouldSaveConfig; the actual write happens below

  WiFi.mode(WIFI_STA);
  wm.setConnectTimeout(30);
  wm.autoConnect(AP_NAME); // Blocks here showing the config portal if no known WiFi network is reachable

  // WiFiManagerParameter keeps its own internal copy; copy the (possibly user-edited) values back
  // into the globals so buildTopics()/client.setServer()/etc. downstream see the final values.
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

// Tries to (re)connect to the broker for up to 30s (retrying every 5s); gives up and returns if
// that window elapses, but since loop() calls reconnect() again whenever the client isn't
// connected, it effectively keeps retrying forever, just in 30s bursts.
void reconnect() {
  unsigned long startAttemptTime = millis();
  while (!client.connected() && millis() - startAttemptTime < 30000) { // 30 seconds timeout
    Serial.print("Attempting MQTT connection...");
    char clientId[15];
    // random() returns a long; cast to unsigned int for %X (harmless on ESP8266/32-bit where
    // both are 4 bytes, but the cast removes the type mismatch regardless of platform).
    snprintf(clientId, sizeof(clientId), "ESPClient-%X", (unsigned int)random(0xffff));
    // Registers connection_topic/"Disconnected" as the Last Will and Testament (retained, QoS 0):
    // if the ESP loses power or network without a clean disconnect, the broker publishes this on
    // our behalf so HA's availability tracking (see publishDiscovery()) still goes "offline".
    if (client.connect(clientId, mqtt_user, mqtt_pass, connection_topic, 0, true, "Disconnected")) {
      Serial.println("connected");
      client.subscribe(cmd_topic);
      client.publish(connection_topic, "Connected", true); // Retained, so HA sees us online immediately on restart
      publishDiscovery(); // Re-announce every entity; harmless/idempotent, and self-heals if HA's discovery cache was cleared
      break;
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      nonBlockingDelay(5000); // Wait before retrying
    }
  }
}

// Dispatches commands received on cmd_topic (subscribed in reconnect()). Payload grammar, each
// mapped from a Home Assistant discovery entity in publishDiscovery():
//   "ON"/"OF"   -> climate mode (heat/off)
//   "P0".."P5"  -> climate preset_mode (flame power level)
//   "F0".."F5"  -> climate fan_mode (fan speed level)
//   "Tnn"       -> climate target temperature, two decimal digits (e.g. "T21")
//   "E"         -> emergency stop button (forces the stove off same as "OF")
// Anything else, or a payload that doesn't match one of these shapes, is silently ignored.
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
    fastUpdate = true; // Switch to the fast polling cadence so the change is reflected quickly in HA
  }
  else if (payload[0] == 'P' && length == 2) {
    int idx = payload[1] - '0'; // Non-digit characters land outside 0-5 and are rejected by the bounds check
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
    // strtol() returns 0 for non-numeric input, which the clamp below also catches (0 < STOVE_MIN_TEMP).
    // The clamp matters even though HA's climate slider is already bounded to [STOVE_MIN_TEMP,
    // STOVE_MAX_TEMP]: anything can publish directly to cmd_topic bypassing that UI, and an
    // out-of-range value written straight to EEPROM here has no other safety net.
    long tempValueRaw = strtol(t, NULL, 10);
    if (tempValueRaw < STOVE_MIN_TEMP) tempValueRaw = STOVE_MIN_TEMP;
    if (tempValueRaw > STOVE_MAX_TEMP) tempValueRaw = STOVE_MAX_TEMP;
    char tempValue = (char)tempValueRaw;
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

// Waits up to `timeout` ms for exactly 2 bytes from the stove (checksum + value) following a read
// request, then decodes and publishes whichever register they correspond to. Called by every
// get*() function below, right after it writes a 2-byte read request and drops ENABLE_RX low.
// If anything other than exactly 2 bytes arrives (partial reply, noise, timeout), the whole
// exchange is discarded - rxCount != 2 falls through with no publish, no error reported.
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
  digitalWrite(ENABLE_RX, HIGH); // Done listening: disable the receiver again regardless of outcome

  if (rxCount == 2) {
    byte val = stoveRxData[1];
    byte checksum = stoveRxData[0];
    byte param = checksum - val; // Recovers "operation code + address" - see the protocol comment above sendCommand()'s frame tables
    switch (param)
    {
      // RAM Cases: overall state machine. Every non-idle/non-transient state also re-publishes
      // onoff_topic (after a 1s delay to let the retained state_topic message land first in HA),
      // so onoff_topic always reflects reality even if it drifts out of sync (e.g. after a stove
      // fault or a command issued from the stove's own physical remote instead of MQTT).
      case stoveStateAddr:
        stoveState = val;
        switch (stoveState)
        {
          case 0: // Off
            client.publish(state_topic, "Spenta", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 1: // Starting up
            client.publish(state_topic, "Avvio", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 2: // Loading pellets into the brazier
            client.publish(state_topic, "Caricamento pellet", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 3: // Ignition in progress
            client.publish(state_topic, "Accensione", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 4: // Running normally
            client.publish(state_topic, "Accesa", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "ON", true);
            break;
          case 5: // Brazier self-cleaning cycle - transient, onoff_topic deliberately left untouched
            client.publish(state_topic, "Pulizia braciere", true);
            break;
          case 6: // Final cleaning cycle before shutting down
            client.publish(state_topic, "Pulizia finale", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 7: // Standby (off but ready)
            client.publish(state_topic, "Standby", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 8: // Out of pellets - alarm-like state, onoff_topic deliberately left untouched
            client.publish(state_topic, "Pellet mancante", true);
            break;
          case 9: // Failed to ignite
            client.publish(state_topic, "Mancata accensione", true);
            nonBlockingDelay(1000);
            client.publish(onoff_topic, "OFF", true);
            break;
          case 10: // Generic alarm/fault - onoff_topic deliberately left untouched
            client.publish(state_topic, "Allarme", true);
            break;
        }
        break;
      case ambTempAddr: // Room temperature, stored on the stove as °C * 2 - hence the / 2 below
        ambTemp = (float)val / 2;
        char ambTempStr[10];
        dtostrf(ambTemp, 4, 2, ambTempStr);
        client.publish(ambtemp_topic, ambTempStr, true);
        break;
      case fumesTempAddr: // Flue gas temperature, already in whole °C
        fumesTemp = val;
        char fumesTempStr[10];
        sprintf(fumesTempStr, "%d", fumesTemp);
        client.publish(fumetemp_topic, fumesTempStr, true);
        break;
      // EEPROM Cases (persisted settings, echoed back after a write so MQTT/HA reflect what the
      // stove actually accepted rather than what was merely requested)
      case tempSetAddr + _EEPROMR: // Thermostat set-point, °C
        tempSet = val;
        char tempSetStr[10];
        sprintf(tempSetStr, "%d", tempSet);
        client.publish(tempset_topic, tempSetStr, true);
        break;
      case powerSetAddr + _EEPROMR: // Flame power level, 0-5 (published on the "power" sub-topic, see flame_topic)
        flamePower = val;
        char flamePowerStr[10];
        sprintf(flamePowerStr, "%d", flamePower);
        client.publish(flame_topic, flamePowerStr, true);
        break;
      case fanSetAddr + _EEPROMR: // Fan speed level, 0-5
        fanSpeed = val;
        char fanSpeedStr[10];
        sprintf(fanSpeedStr, "%d", fanSpeed);
        client.publish(fan_topic, fanSpeedStr, true);
        break;
    }
  }
}

// Each get*() below follows the same read sequence: write the 2-byte request (operation code +
// register address), drop ENABLE_RX low to start listening, wait long enough for the stove to
// respond (80ms), then let checkStoveReply() collect and publish the answer.
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

// Polls every register in sequence (100ms apart, to give the stove breathing room between
// exchanges) and finally re-publishes connectivity. Called from loop() on every update cycle,
// wrapped in the sem guard so it can't overlap with a command triggered from callback().
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
  client.publish(connection_topic, "Connected", true);
}

void setup() {
  // pinMode() must run before the digitalWrite() below: calling digitalWrite() on a pin that
  // hasn't been explicitly configured yet leaves it in an indeterminate state for however long
  // it takes the rest of setup() to reach the old pinMode() call - and setup_wifi() alone can
  // block for up to 30s (or indefinitely if it has to show the config portal on first boot).
  pinMode(ENABLE_RX, OUTPUT);
  digitalWrite(ENABLE_RX, HIGH); // Start with the receiver disabled; only pulled low right before each read exchange
  Serial.begin(115200);
  StoveSerial.begin(1200, SERIAL_MODE, RX_PIN, TX_PIN, false, 256);

  loadConfig();   // Pre-fill defaults from LittleFS, if a config was already saved
  setup_wifi();   // Connects to WiFi; shows the config portal (with MQTT fields) if needed
  buildTopics();  // mqtt_topic is only known now, so build the full topic strings (and node_id)

  ArduinoOTA.setHostname(node_id[0] ? node_id : AP_NAME); // node_id has no '/', unlike mqtt_topic, so it's always a valid mDNS hostname
  ArduinoOTA.setPassword("micronova");
  ArduinoOTA.begin();

  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setCallback(callback);
  client.setBufferSize(2048); // Sized once here (not on every reconnect) for the retained HA discovery payloads
}

void loop() {
  if (!client.connected()) reconnect(); // Also (re)subscribes and re-publishes discovery on success
  client.loop();     // Processes incoming MQTT packets (-> callback()) and sends the keep-alive
  ArduinoOTA.handle(); // Services pending OTA update requests

  // Runs getStates() either right after boot (`boot` is only true once) or every updatePeriod;
  // updatePeriod itself shortens to FAST_UPDATE_PERIOD for FAST_LOOP_CYCLES polls after any
  // command (fastUpdate is set in callback()), so HA sees the stove react quickly, then eases
  // back to the slow cadence once things have settled.
  unsigned long updatePeriod = fastUpdate ? FAST_UPDATE_PERIOD : UPDATE_PERIOD;
  if ((millis() - previousMillis > updatePeriod) || boot) {
    boot = false;
    previousMillis = millis();
    if (!sem) { // Skip this cycle entirely if a command is already using the UART (see sem's declaration)
      setSemaphore(true);
      getStates();
      setSemaphore(false);
    }
    client.publish(connection_topic, "Connected", true); // Refresh the retained availability message even if getStates() was skipped above
    if (fastUpdate) loopCounter++;
    if (loopCounter > FAST_LOOP_CYCLES) { fastUpdate = false; loopCounter = 0; } // Back to the slow cadence
  }
}