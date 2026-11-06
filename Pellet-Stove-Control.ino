/*
 * Pellet Stove Control - MQTT bridge for Micronova stove controllers (tested on the N100).
 *
 * Stove bus: single-wire serial, 1200 baud 8N2, behind the optocoupler interface shown in
 * PCB/Schematics.png. RX_PIN only hears the bus while ENABLE_RX is LOW, so the receiver is
 * enabled just while waiting for a reply and never sees our own transmissions.
 *
 * Protocol (see docs/micronova_stove_commands_mapping.xlsx):
 *   read   ESP -> [opcode, address]                  stove -> [checksum, value]
 *   write  ESP -> [opcode, address, value, checksum]
 *   checksum = opcode + address + value (mod 256)
 *
 * MQTT topics, below the base topic set in the config portal (see README.md):
 *   cmd (subscribed): ON, OFF, P0-P5, F0-F5, Txx, E
 *   state, onoff, ambtemp, tempset, fumetemp, power, fan (retained), connection (not retained)
 *   status (retained): online / offline (last will), availability of the board
 *
 * Home Assistant: MQTT discovery configs are published (retained) below the discovery prefix
 * at every connection, and again whenever Home Assistant comes back online. The entities they
 * describe use the topics above, so the topics stay the same for every other integration.
 */

#include <stdarg.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <LittleFS.h>

// ---------------------------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------------------------

#define FW_VERSION   "2.1.0"
#define AP_NAME      "PelletStove-Setup" // Access point of the config portal
#define CONFIG_FILE  "/mqtt_config.txt"  // MQTT settings on LittleFS, one per line
#define OTA_PASSWORD "micronova"

#define HA_DEFAULT_PREFIX "homeassistant" // Discovery prefix used when the portal field is empty
#define HA_DEVICE_NAME    "Pellet Stove"  // Device name in Home Assistant (renamable there)

const unsigned long UPDATE_PERIOD      = 1800000; // 30 min between two polls...
const unsigned long FAST_UPDATE_PERIOD = 60000;   // ...1 min after a command,
const uint8_t       FAST_LOOP_CYCLES   = 30;      // for this many polls

const unsigned long PORTAL_TIMEOUT_S = 180; // Restart when the config portal is left idle

// Target temperature range for the Txx command (°C): values outside are clamped. Macros, since
// they also go in the Home Assistant discovery config. Both must have two digits (Txx).
#define TEMP_SET_MIN 10
#define TEMP_SET_MAX 40

#if defined(ESP32)
// ESP32 DevKit. ENABLE_RX must be output capable: GPIO34-39 are input only.
const uint8_t RX_PIN    = 32;
const uint8_t TX_PIN    = 33;
const uint8_t ENABLE_RX = 25;
#else
// Wemos D1 mini, as on the project PCB
const uint8_t RX_PIN    = D3; // GPIO0
const uint8_t TX_PIN    = D4; // GPIO2
const uint8_t ENABLE_RX = D2; // GPIO4, LOW = receiver enabled
#endif

// ---------------------------------------------------------------------------------------------
// Stove protocol
// ---------------------------------------------------------------------------------------------

// Opcodes: memory area to read from, or to write to
const uint8_t RAM_READ     = 0x00;
const uint8_t RAM_WRITE    = 0x80;
const uint8_t EEPROM_READ  = 0x20;
const uint8_t EEPROM_WRITE = 0xA0;

// RAM addresses
const uint8_t stoveStateAddr  = 0x21; // Stove state, see STOVE_STATES
const uint8_t ambTempAddr     = 0x01; // Ambient temperature, 0.5 °C steps
const uint8_t fumesTempAddr   = 0x3E; // Flue gas temperature, °C
const uint8_t stovePOnOffAddr = 0x58; // Writing 0x5A here works like the power button

// EEPROM addresses
const uint8_t powerSetAddr = 0x89; // Flame power, 0-5
const uint8_t fanSetAddr   = 0x8A; // Fan speed, 0-5 (0 = auto)
const uint8_t tempSetAddr  = 0x8B; // Target temperature, °C

// Bus timings (ms)
const unsigned long BYTE_GAP_MS      = 1;   // Between the bytes we send
const unsigned long REPLY_DELAY_MS   = 80;  // Listening before collecting a reply...
const unsigned long REPLY_TIMEOUT_MS = 200; // ...then waiting at most this for its 2 bytes
const unsigned long WRITE_SETTLE_MS  = 50;  // After a write frame
const unsigned long READ_GAP_MS      = 100; // After each register read

// ---------------------------------------------------------------------------------------------
// MQTT
// ---------------------------------------------------------------------------------------------

// Sub-topics, appended to the base topic as "<base>/<sub-topic>"
#define TOPIC_CMD        "cmd"
#define TOPIC_CONNECTION "connection" // Longest one, sizes the topic buffers
#define TOPIC_STATE      "state"
#define TOPIC_ONOFF      "onoff"
#define TOPIC_AMBTEMP    "ambtemp"
#define TOPIC_TEMPSET    "tempset"
#define TOPIC_FUMETEMP   "fumetemp"
#define TOPIC_POWER      "power"
#define TOPIC_FAN        "fan"
#define TOPIC_STATUS     "status"     // "online", or "offline" as last will (both retained)

#define STATUS_ONLINE  "online"
#define STATUS_OFFLINE "offline"

// Stove states: text published on "state" and the matching "onoff" value (nullptr = none)
struct StoveState { const char *text; const char *onoff; };
const StoveState STOVE_STATES[] = {
  {"Spenta",             "OFF"},   // 0  Off
  {"Avvio",              "ON"},    // 1  Start-up
  {"Caricamento pellet", "ON"},    // 2  Pellet loading
  {"Accensione",         "ON"},    // 3  Ignition
  {"Accesa",             "ON"},    // 4  Running
  {"Pulizia braciere",   nullptr}, // 5  Brazier cleaning
  {"Pulizia finale",     "OFF"},   // 6  Final cleaning
  {"Standby",            "OFF"},   // 7  Standby
  {"Pellet mancante",    nullptr}, // 8  Out of pellets
  {"Mancata accensione", "OFF"},   // 9  Ignition failure
  {"Allarme",            nullptr}, // 10 Alarm
};
const unsigned long ONOFF_DELAY_MS = 1000; // Between publishing "state" and "onoff"

// Registers read by every poll, in this order
enum ValueFormat : uint8_t { STATE_TEXT, HALF_DEGREES, INTEGER };
struct PolledRegister { uint8_t opcode, address; const char *topic; ValueFormat format; };
const PolledRegister POLLED_REGISTERS[] = {
  {RAM_READ,    stoveStateAddr, TOPIC_STATE,    STATE_TEXT},
  {RAM_READ,    ambTempAddr,    TOPIC_AMBTEMP,  HALF_DEGREES},
  {EEPROM_READ, tempSetAddr,    TOPIC_TEMPSET,  INTEGER},
  {RAM_READ,    fumesTempAddr,  TOPIC_FUMETEMP, INTEGER},
  {EEPROM_READ, powerSetAddr,   TOPIC_POWER,    INTEGER},
  {EEPROM_READ, fanSetAddr,     TOPIC_FAN,      INTEGER},
};

// ---------------------------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------------------------

// MQTT settings, from the config portal or CONFIG_FILE
char mqtt_server[41] = "";
char mqtt_port[7]    = "1883";
char mqtt_topic[41]  = ""; // Base topic
char mqtt_user[65]   = "";
char mqtt_pass[65]   = "";
char ha_prefix[33]   = HA_DEFAULT_PREFIX; // Home Assistant discovery prefix, empty = default

// Topics subscribed to, compared in callback()
char cmdTopic[sizeof(mqtt_topic) + sizeof("/" TOPIC_CMD)];
// Home Assistant birth/will. Its topic is a setting of its own in Home Assistant: the default
// one is listened to as well, in case only the discovery prefix was changed.
#define HA_DEFAULT_STATUS_TOPIC HA_DEFAULT_PREFIX "/status"
char haStatusTopic[sizeof(ha_prefix) + sizeof("/status")]; // "<prefix>/status"

char deviceId[13] = ""; // MAC address in hex: unique and stable id of this board

SoftwareSerial StoveSerial;
WiFiClient espClient;
PubSubClient client(espClient);

// Commands from MQTT wait here until loop() sends them: callback() may run in the middle of
// a stove exchange (see nonBlockingDelay()), when the bus is busy
struct StoveCommand { uint8_t opcode, address, value; const char *onoff; };
const uint8_t CMD_QUEUE_SIZE = 8;
StoveCommand cmdQueue[CMD_QUEUE_SIZE];
uint8_t cmdHead = 0, cmdCount = 0;

unsigned long lastPollMillis = 0;
uint8_t fastPollsLeft = 0;     // Polls still due at the fast rate
bool pollPending = false;      // Poll at once (set on every MQTT connection)
bool discoveryPending = false; // Publish the discovery configs (connection, Home Assistant restart)
bool shouldSaveConfig = false; // Set whenever the config portal form is submitted

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

// Waits while keeping MQTT and OTA alive. Incoming commands are queued, see callback().
void nonBlockingDelay(unsigned long ms) {
  const unsigned long start = millis();
  while (millis() - start < ms) {
    client.loop();
    ArduinoOTA.handle();
    yield();
  }
}

// Copies src into dst (size bytes) without leading/trailing whitespace
void copyTrimmed(char *dst, const char *src, size_t size) {
  while (isspace((unsigned char)*src)) src++;
  size_t len = strlen(src);
  while (len > 0 && isspace((unsigned char)src[len - 1])) len--;
  if (len >= size) len = size - 1;
  memcpy(dst, src, len);
  dst[len] = '\0';
}

// Publishes on "<base>/<subTopic>"
void publish(const char *subTopic, const char *payload, bool retained = true) {
  char topic[sizeof(mqtt_topic) + sizeof("/" TOPIC_CONNECTION)];
  snprintf(topic, sizeof(topic), "%s/%s", mqtt_topic, subTopic);
  client.publish(topic, payload, retained);
}

// ---------------------------------------------------------------------------------------------
// Stove I/O
// ---------------------------------------------------------------------------------------------

void writeRegister(uint8_t opcode, uint8_t address, uint8_t value) {
  const uint8_t frame[] = {opcode, address, value, (uint8_t)(opcode + address + value)};
  for (uint8_t b : frame) {
    StoveSerial.write(b);
    delay(BYTE_GAP_MS); // Plain delay: nothing else runs in the middle of a frame
  }
  nonBlockingDelay(WRITE_SETTLE_MS);
}

// Returns the register value, or -1 if no valid reply arrived
int readRegister(uint8_t opcode, uint8_t address) {
  while (StoveSerial.available()) StoveSerial.read(); // Drop stale bytes (late replies, noise)

  StoveSerial.write(opcode);
  delay(BYTE_GAP_MS);
  StoveSerial.write(address);
  digitalWrite(ENABLE_RX, LOW);
  nonBlockingDelay(REPLY_DELAY_MS);

  uint8_t reply[2]; // [checksum, value]
  uint8_t count = 0;
  const unsigned long start = millis();
  while (count < 2 && millis() - start < REPLY_TIMEOUT_MS) {
    if (StoveSerial.available()) reply[count++] = StoveSerial.read();
    else yield();
  }
  digitalWrite(ENABLE_RX, HIGH);

  // The checksum must match this very register, otherwise the reply is not ours
  if (count < 2 || (uint8_t)(reply[0] - reply[1]) != (uint8_t)(opcode + address)) return -1;
  return reply[1];
}

void publishValue(const PolledRegister &reg, uint8_t value) {
  char text[8];
  switch (reg.format) {
    case STATE_TEXT:
      if (value >= sizeof(STOVE_STATES) / sizeof(STOVE_STATES[0])) return; // Unknown state
      publish(reg.topic, STOVE_STATES[value].text);
      if (STOVE_STATES[value].onoff) {
        nonBlockingDelay(ONOFF_DELAY_MS);
        publish(TOPIC_ONOFF, STOVE_STATES[value].onoff);
      }
      return;
    case HALF_DEGREES: // e.g. 41 -> "20.50"
      snprintf(text, sizeof(text), "%d.%s", value / 2, (value & 1) ? "50" : "00");
      break;
    default:
      snprintf(text, sizeof(text), "%d", value);
      break;
  }
  publish(reg.topic, text);
}

void pollStove() {
  for (const PolledRegister &reg : POLLED_REGISTERS) {
    const int value = readRegister(reg.opcode, reg.address);
    if (value >= 0) publishValue(reg, value);
    nonBlockingDelay(READ_GAP_MS);
  }
  publish(TOPIC_CONNECTION, "Connected", false);
}

// ---------------------------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------------------------

// onoff ("ON"/"OFF") is published on the onoff topic once the command has been sent
void queueCommand(uint8_t opcode, uint8_t address, uint8_t value, const char *onoff = nullptr) {
  if (cmdCount == CMD_QUEUE_SIZE) {
    Serial.println(F("Command queue full, command dropped"));
    return;
  }
  cmdQueue[(cmdHead + cmdCount) % CMD_QUEUE_SIZE] = {opcode, address, value, onoff};
  cmdCount++;
  fastPollsLeft = FAST_LOOP_CYCLES; // Follow the stove closely for a while
}

void sendQueuedCommands() {
  while (cmdCount > 0) {
    const StoveCommand cmd = cmdQueue[cmdHead];
    cmdHead = (cmdHead + 1) % CMD_QUEUE_SIZE;
    cmdCount--;
    writeRegister(cmd.opcode, cmd.address, cmd.value); // May queue further commands meanwhile
    if (cmd.onoff) publish(TOPIC_ONOFF, cmd.onoff);
  }
}

// Handles "<base>/cmd" (invalid commands are ignored) and the Home Assistant status topic
void callback(char *topic, byte *payload, unsigned int length) {
  if (strcmp(topic, haStatusTopic) == 0 || strcmp(topic, HA_DEFAULT_STATUS_TOPIC) == 0) {
    // Home Assistant (re)started: send the configs again, in case the broker lost them.
    // Published from loop(): the client buffer still holds this message.
    if (length == 6 && memcmp(payload, STATUS_ONLINE, 6) == 0) discoveryPending = true;
    return;
  }
  if (strcmp(topic, cmdTopic) != 0 || length == 0) return;
  const uint8_t arg = length > 1 ? payload[1] : 0;

  switch (payload[0]) {
    case 'O': // ON / OFF (only the first two characters matter)
      if (arg == 'N') queueCommand(RAM_WRITE, stovePOnOffAddr, 0x5A, "ON");
      else if (arg == 'F') queueCommand(RAM_WRITE, stoveStateAddr, 0x00, "OFF");
      break;
    case 'P': // P0-P5: flame power
    case 'F': // F0-F5: fan speed
      if (length == 2 && arg >= '0' && arg <= '5')
        queueCommand(EEPROM_WRITE, payload[0] == 'P' ? powerSetAddr : fanSetAddr, arg - '0');
      break;
    case 'T': // Txx: target temperature, two digits
      if (length == 3 && isdigit(arg) && isdigit(payload[2])) {
        const int temp = (arg - '0') * 10 + (payload[2] - '0');
        queueCommand(EEPROM_WRITE, tempSetAddr, constrain(temp, TEMP_SET_MIN, TEMP_SET_MAX));
      }
      break;
    case 'E': // Error reset: same frame as OFF
      queueCommand(RAM_WRITE, stoveStateAddr, 0x00, "OFF");
      break;
  }
}

// ---------------------------------------------------------------------------------------------
// Home Assistant MQTT discovery
// ---------------------------------------------------------------------------------------------

#define STR_(x) #x
#define STR(x)  STR_(x)

// One retained discovery config per entity, on "<prefix>/<component>/pellet_stove_<id>/<object>/config".
// members holds the entity specific JSON members: "~" is replaced by the base topic, so every
// entity works on the topics listed at the top. publishDiscovery() adds the common members.
struct DiscoveryEntity {
  const char *component, *object, *members;
  bool stateOptions = false;    // Append the STOVE_STATES texts as "options" (enum sensor)
  bool alwaysAvailable = false; // No availability topic: the entity also works while offline
};
const DiscoveryEntity DISCOVERY_ENTITIES[] = {
  // Thermostat card: on/off, target and ambient temperature, fan speed, what the stove is doing
  {"climate", "thermostat",
   R"J("name":null,"icon":"mdi:fireplace","modes":["off","heat"],)J"
   R"J("mode_command_topic":"~/cmd","mode_command_template":"{{ 'ON' if value == 'heat' else 'OFF' }}",)J"
   R"J("mode_state_topic":"~/onoff","mode_state_template":"{{ 'heat' if value == 'ON' else 'off' }}",)J"
   R"J("temperature_command_topic":"~/cmd","temperature_command_template":"T{{ value | int }}",)J"
   R"J("temperature_state_topic":"~/tempset","current_temperature_topic":"~/ambtemp",)J"
   R"J("min_temp":)J" STR(TEMP_SET_MIN) R"J(,"max_temp":)J" STR(TEMP_SET_MAX)
   R"J(,"temp_step":1,"precision":0.5,"temperature_unit":"C",)J"
   R"J("fan_modes":["auto","1","2","3","4","5"],"fan_mode_command_topic":"~/cmd",)J"
   R"J("fan_mode_command_template":"F{{ 0 if value == 'auto' else value }}",)J"
   R"J("fan_mode_state_topic":"~/fan","fan_mode_state_template":"{{ 'auto' if value == '0' else value }}",)J"
   R"J("action_topic":"~/state","action_template":"{{ {'Spenta': 'off', 'Avvio': 'preheating', )J"
   R"J('Caricamento pellet': 'preheating', 'Accensione': 'preheating', 'Accesa': 'heating'})J"
   R"J(.get(value, 'idle') }}")J"},
  // Target temperature on its own, e.g. for entities cards and automations
  {"number", "target_temperature",
   R"J("name":"Target temperature","icon":"mdi:thermometer","device_class":"temperature",)J"
   R"J("unit_of_measurement":"°C","min":)J" STR(TEMP_SET_MIN) R"J(,"max":)J" STR(TEMP_SET_MAX)
   R"J(,"step":1,"state_topic":"~/tempset","command_topic":"~/cmd",)J"
   R"J("command_template":"T{{ value | int }}")J"},
  // On/off, e.g. for schedulers and automations
  {"switch", "power",
   R"J("name":"Power","icon":"mdi:power","state_topic":"~/onoff","command_topic":"~/cmd",)J"
   R"J("payload_on":"ON","payload_off":"OFF")J"},
  // Flame power and fan speed: level 0 is automatic
  {"select", "power_level",
   R"J("name":"Power level","icon":"mdi:fire","options":["Auto","1","2","3","4","5"],)J"
   R"J("state_topic":"~/power","value_template":"{{ 'Auto' if value == '0' else value }}",)J"
   R"J("command_topic":"~/cmd","command_template":"P{{ 0 if value == 'Auto' else value }}")J"},
  {"select", "fan_speed",
   R"J("name":"Fan speed","icon":"mdi:fan","options":["Auto","1","2","3","4","5"],)J"
   R"J("state_topic":"~/fan","value_template":"{{ 'Auto' if value == '0' else value }}",)J"
   R"J("command_topic":"~/cmd","command_template":"F{{ 0 if value == 'Auto' else value }}")J"},
  {"sensor", "state",
   R"J("name":"State","icon":"mdi:fire-alert","state_topic":"~/state","device_class":"enum")J",
   true},
  {"sensor", "ambient_temperature",
   R"J("name":"Ambient temperature","state_topic":"~/ambtemp","device_class":"temperature",)J"
   R"J("state_class":"measurement","unit_of_measurement":"°C","suggested_display_precision":1)J"},
  {"sensor", "fumes_temperature",
   R"J("name":"Fumes temperature","icon":"mdi:thermometer-high","state_topic":"~/fumetemp",)J"
   R"J("device_class":"temperature","state_class":"measurement","unit_of_measurement":"°C",)J"
   R"J("suggested_display_precision":0)J"},
  // Error reset: same frame as OFF
  {"button", "error_reset",
   R"J("name":"Error reset","icon":"mdi:restart","command_topic":"~/cmd","payload_press":"E",)J"
   R"J("entity_category":"config")J"},
  // Board connected to the broker or not: the status topic itself, so never unavailable
  {"binary_sensor", "connection",
   R"J("name":"Connection","device_class":"connectivity","entity_category":"diagnostic",)J"
   R"J("state_topic":"~/status","payload_on":"online","payload_off":"offline")J",
   false, true},
};

// Payloads are streamed to the broker (beginPublish), so they don't need to fit the PubSubClient
// buffer (256 bytes). The longest one, the thermostat, takes about 1.3 kB.
char discoveryBuf[1536];
size_t discoveryLen; // >= sizeof(discoveryBuf) once the payload doesn't fit

// Appends to discoveryBuf, printf style
void appendDiscovery(const char *format, ...) {
  if (discoveryLen >= sizeof(discoveryBuf)) return;
  const size_t room = sizeof(discoveryBuf) - discoveryLen;
  va_list args;
  va_start(args, format);
  const int n = vsnprintf(discoveryBuf + discoveryLen, room, format, args);
  va_end(args);
  discoveryLen = (n < 0 || (size_t)n >= room) ? sizeof(discoveryBuf) : discoveryLen + n;
}

void publishDiscovery() {
  const char *prefix = ha_prefix[0] ? ha_prefix : HA_DEFAULT_PREFIX;
  for (const DiscoveryEntity &entity : DISCOVERY_ENTITIES) {
    discoveryLen = 0;
    appendDiscovery("{\"~\":\"%s\",\"unique_id\":\"pellet_stove_%s_%s\",%s%s",
                    mqtt_topic, deviceId, entity.object,
                    entity.alwaysAvailable ? "" : "\"availability_topic\":\"~/" TOPIC_STATUS "\",",
                    entity.members);
    if (entity.stateOptions) {
      appendDiscovery(",\"options\":[");
      for (size_t i = 0; i < sizeof(STOVE_STATES) / sizeof(STOVE_STATES[0]); i++)
        appendDiscovery("%s\"%s\"", i ? "," : "", STOVE_STATES[i].text);
      appendDiscovery("]");
    }
    appendDiscovery(",\"device\":{\"identifiers\":[\"pellet_stove_%s\"],\"name\":\"" HA_DEVICE_NAME "\","
                    "\"model\":\"Pellet Stove Control\",\"sw_version\":\"" FW_VERSION "\"},"
                    "\"origin\":{\"name\":\"Pellet-Stove-Control\",\"sw_version\":\"" FW_VERSION "\","
                    "\"support_url\":\"https://github.com/fabrizioromanelli/Pellet-Stove-Control\"}}",
                    deviceId);
    if (discoveryLen >= sizeof(discoveryBuf)) {
      Serial.print(F("Discovery config too long, skipped: "));
      Serial.println(entity.object);
      continue;
    }

    char topic[sizeof(ha_prefix) + 80];
    snprintf(topic, sizeof(topic), "%s/%s/pellet_stove_%s/%s/config",
             prefix, entity.component, deviceId, entity.object);
    if (!client.beginPublish(topic, discoveryLen, true)) return; // Connection lost
    client.write((const uint8_t *)discoveryBuf, discoveryLen);
    client.endPublish();
  }
}

// ---------------------------------------------------------------------------------------------
// Connectivity
// ---------------------------------------------------------------------------------------------

// The MQTT settings, in CONFIG_FILE line order. New settings go at the end: the files saved by
// older firmware versions lack their lines, which are then read as empty.
struct ConfigField { char *value; size_t size; };
const ConfigField CONFIG_FIELDS[] = {
  {mqtt_server, sizeof(mqtt_server)},
  {mqtt_port,   sizeof(mqtt_port)},
  {mqtt_topic,  sizeof(mqtt_topic)},
  {mqtt_user,   sizeof(mqtt_user)},
  {mqtt_pass,   sizeof(mqtt_pass)},
  {ha_prefix,   sizeof(ha_prefix)},
};
const size_t CONFIG_FIELD_COUNT = sizeof(CONFIG_FIELDS) / sizeof(CONFIG_FIELDS[0]);

void loadConfig() {
  if (!LittleFS.exists(CONFIG_FILE)) return;
  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return;
  for (const ConfigField &field : CONFIG_FIELDS)
    copyTrimmed(field.value, f.readStringUntil('\n').c_str(), field.size);
  f.close();
}

void saveConfig() {
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return;
  for (const ConfigField &field : CONFIG_FIELDS) f.println(field.value);
  f.close();
}

// Connects to WiFi. The config portal, which also asks for the MQTT settings (pre-filled with
// the saved ones), opens when no saved network is reachable or the MQTT settings are missing.
void setupWifi() {
  // Same order as CONFIG_FIELDS. The portal is an open access point, so the saved password is
  // never shown: an empty password field keeps it.
  WiFiManagerParameter fields[] = {
    {"server", "MQTT server",     mqtt_server, sizeof(mqtt_server) - 1},
    {"port",   "MQTT port",       mqtt_port,   sizeof(mqtt_port) - 1},
    {"topic",  "MQTT base topic", mqtt_topic,  sizeof(mqtt_topic) - 1},
    {"user",   "MQTT user",       mqtt_user,   sizeof(mqtt_user) - 1},
    {"pass",   "MQTT password (empty = keep the saved one)", "", sizeof(mqtt_pass) - 1,
     "type=\"password\""},
    {"haprefix", "Home Assistant discovery prefix (empty = " HA_DEFAULT_PREFIX ")",
     ha_prefix, sizeof(ha_prefix) - 1},
  };
  static_assert(sizeof(fields) / sizeof(fields[0]) == CONFIG_FIELD_COUNT, "one field per setting");

  WiFiManager wm; // Only needed here: released on return
  for (WiFiManagerParameter &field : fields) wm.addParameter(&field);
  wm.setSaveParamsCallback([] { shouldSaveConfig = true; }); // On every form submit
  wm.setConnectTimeout(30);
  // Without a timeout the portal would wait forever, e.g. when the router is still booting
  // after a power cut: restart instead and try the saved network again
  wm.setConfigPortalTimeout(PORTAL_TIMEOUT_S);

  WiFi.mode(WIFI_STA);
  // Server and base topic are mandatory: ask for them even if the WiFi network is known
  // (e.g. boards updated from the firmware versions with hardcoded MQTT settings)
  const bool connected = mqtt_server[0] && mqtt_topic[0] ? wm.autoConnect(AP_NAME)
                                                         : wm.startConfigPortal(AP_NAME);

  for (size_t i = 0; i < CONFIG_FIELD_COUNT; i++) {
    const char *value = fields[i].getValue();
    if (CONFIG_FIELDS[i].value == mqtt_pass && !value[0]) continue; // Keep the saved password
    copyTrimmed(CONFIG_FIELDS[i].value, value, CONFIG_FIELDS[i].size);
  }
  if (shouldSaveConfig) saveConfig();

  if (!connected) {
    Serial.println(F("No WiFi, restarting"));
    ESP.restart();
  }
}

// Tries to connect to the broker for up to 30 s, every 5 s. Returns true once connected.
bool reconnect() {
  const unsigned long start = millis();
  do {
    Serial.print(F("Attempting MQTT connection..."));
    char clientId[15];
    snprintf(clientId, sizeof(clientId), "ESPClient-%X", (unsigned)random(0xffff));
    // Credentials are sent only when a user is configured. The broker publishes the last will
    // ("offline" on the status topic) when the board drops off.
    const char *user = mqtt_user[0] ? mqtt_user : nullptr;
    char statusTopic[sizeof(mqtt_topic) + sizeof("/" TOPIC_STATUS)];
    snprintf(statusTopic, sizeof(statusTopic), "%s/" TOPIC_STATUS, mqtt_topic);
    if (client.connect(clientId, user, user ? mqtt_pass : nullptr,
                       statusTopic, 0, true, STATUS_OFFLINE)) {
      Serial.println(F("connected"));
      client.subscribe(cmdTopic);
      client.subscribe(haStatusTopic);
      if (strcmp(haStatusTopic, HA_DEFAULT_STATUS_TOPIC) != 0) client.subscribe(HA_DEFAULT_STATUS_TOPIC);
      publish(TOPIC_STATUS, STATUS_ONLINE);
      return true;
    }
    Serial.print(F("failed, rc="));
    Serial.println(client.state());
    nonBlockingDelay(5000);
  } while (millis() - start < 30000);
  return false;
}

// ---------------------------------------------------------------------------------------------

void setup() {
  pinMode(ENABLE_RX, OUTPUT);
  digitalWrite(ENABLE_RX, HIGH); // Receiver disabled until a reply is expected
  Serial.begin(115200);
  // RX only collects the 2-byte replies, so a small buffer is plenty
  StoveSerial.begin(1200, SWSERIAL_8N2, RX_PIN, TX_PIN, false, 64);

#if defined(ESP32)
  const bool fsMounted = LittleFS.begin(true); // Formats the partition on first use
#else
  const bool fsMounted = LittleFS.begin();     // Formats automatically when needed
#endif
  if (fsMounted) loadConfig();
  else Serial.println(F("LittleFS not available: MQTT settings won't be saved"));

  setupWifi();

  // OTA host name: the base topic, without the '/' host names can't contain
  char hostname[sizeof(mqtt_topic)];
  strlcpy(hostname, mqtt_topic[0] ? mqtt_topic : AP_NAME, sizeof(hostname));
  for (char *p = hostname; *p; p++) if (*p == '/') *p = '_';
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();

  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(deviceId, sizeof(deviceId), "%02x%02x%02x%02x%02x%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  snprintf(cmdTopic, sizeof(cmdTopic), "%s/" TOPIC_CMD, mqtt_topic);
  snprintf(haStatusTopic, sizeof(haStatusTopic), "%s/status",
           ha_prefix[0] ? ha_prefix : HA_DEFAULT_PREFIX);

  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setCallback(callback);
}

void loop() {
  if (!client.connected() && reconnect()) {
    discoveryPending = true; // Refresh the discovery configs...
    pollPending = true;      // ...and every topic
  }
  client.loop(); // Delivers MQTT messages to callback()
  ArduinoOTA.handle();

  if (discoveryPending && client.connected()) {
    discoveryPending = false;
    publishDiscovery();
  }

  sendQueuedCommands();

  const unsigned long period = fastPollsLeft ? FAST_UPDATE_PERIOD : UPDATE_PERIOD;
  if (client.connected() && (pollPending || millis() - lastPollMillis > period)) {
    pollPending = false;
    lastPollMillis = millis();
    if (fastPollsLeft) fastPollsLeft--;
    pollStove();
  }
}
