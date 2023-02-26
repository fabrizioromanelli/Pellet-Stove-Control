# Pellet Stove Control ![ESP8266](https://img.shields.io/badge/ESP-8266-000000.svg?longCache=true&style=flat&colorA=CCCC33) ![ESP32](https://img.shields.io/badge/ESP-32-000000.svg?longCache=true&style=flat&colorA=FF3500)

**I have prebuilt kits, ask me if you need any information!**

This project allows you to smart control any pellet stove controlled via a Micronova Controller.
The program is written for Arduino, and it is tested on a D1 Mini with full integration for MQTT (Home Assistant, OpenHAB).

## The Pellet Stove Control works with
the following pellet stoves (please ask me before to check if this could work with your
pellet stove!).
<details>
<summary><b>Compatible pellet stoves</b></summary>

- AMG S.p.A. (Artel, Kalor, Tepor, Foco, Adler)
- Anselmo Cola
- L'Asolana
- Boreal
- Bronpi
- Cadel
- Clam
- Corisit S.r.l. (Lincar, Vulcania, Arce)
- Ecoteck
- EL.FIRE
- EOSS
- EvaCalor
- Fontana Calore
- Fonte Flamme
- Galletti
- Globe-Fire
- Italiana Camini
- Jolly Mec
- Karmek
- Kepo
- Klover
- Laminox
- LMX
- La Nordica S.p.A (Extraflame, Dal Zotto)
- Lorflam
- MCZ (Brisach, Cadel, FreePoint, Pegaso, Red)
- Moretti Design
- Nordic Fire
- Piazetta
- Sicalor
- Solius
- Stufe a pellet Italia
- Tecnoflam
- Termoflam
- Thermoflux
- TS
- Ungaro
- Zibro
- Kalor Compact (boiler, fumesTempAddr 0xFF, flamePowerAddr 0x37)
- ITC Layma (boiler, fumesTempAddr 0x5A, no water pressure value available)
- ETNA Giulia EVO (Micronova J042, fumes temp and power reading doesn't work for the moment)
- Extraflame Teodora Evo (at least turning on works)
- El.Fire Venice Slim
- Palazzetti Ecofire Violetta 7 (tested)
- RED Loto
- MCZ Kaika/Face
- FreePoint Sharp
- Piazetta P958D
- Piazetta Line
- Anselmo Cola Aloe
- Solius Geres
- L'Asolana Marina
</details>

## Components
The following components are needed:
- An ESP8266 (D1 Mini or similar).
- 3 x PC817.
- 3 x 510 Ohms resistors.
- 1 x RJ11 Connector (if your stove has an RJ11 connected to the serial interface).

## Circuit diagram
![Schematics](https://user-images.githubusercontent.com/26959336/217276025-cc22cac3-172c-4b23-bbed-d59402fb56fb.png)

For ESP8266 the IN/OUT matrix is:

|ESP8266 I/O|Function|
|---|---|
|`D2`|`Enable RX`|
|`D3`|`RX`|
|`D4`|`TX`|

For ESP32 the IN/OUT matrix is:

|ESP32 I/O|Function|
|---|---|
|`GPIO35`|`Enable RX`|
|`GPIO32`|`RX`|
|`GPIO33`|`TX`|

## PCB project
The folder PCB contains the project for the Printed Circuit Board of the Pellet Stove Control.

Front PCB board:

![PCB Front](https://user-images.githubusercontent.com/26959336/221350145-703fa86c-7866-4b43-a54f-4f3aa0b211a6.jpg)

Back PCB board:

![PCB Retro](https://user-images.githubusercontent.com/26959336/221350151-4c8e91c2-f829-4022-836e-27dc6b2e277f.jpg)

## Micronova board pinout
For the details related to the board pinout, refer to the manual. This project allows you to interface the stove with the serial port of the Micronova controller. Ask me in case of any doubts.

## Protocol for Micronova controller interface
The protocol for the Micronova controller has been reverse engineered and it is as stable as possible; there is a known problem with writing some values for controlling the stove (such as Flame Power and Fan Speed) that are mapped on the EEPROM (rather than on RAM), however this has been included in the project to let the user control all the stove parameters.

## Building the project
- Modify the following parameters contained in the Pellet-Stove-Control.ino
  - #define mqtt_server "your_MQTT_server_ip"
  - #define mqtt_port your_MQTT_port (usually 1883)
  - #define mqtt_topic "topic_name" (the name of the topic you choose to publish on)
  - #define mqtt_user "MQTT_user_name" (if any)
  - #define mqtt_pass "MQTT_password" (if any)
- Build the sketch with the Arduino IDE
- Upload the sketch on the board
- Connect to the `micronova` WiFi network
- The browser should redirect to the configuration page where you can setup your credentials

## MQTT topics
- `mqtt_topic`, this is the main topic.
- `connection_state`, this topic contains connection details.
- `state`, this topic contains the detailed status of the stove.
- `power_state`, this topic contains information about the stove power on/off.
- `ta_state`, this topic contains the ambient temperature.
- `ts_state` this topic is used to read the target temperature.
- `tf_state`, this topic is used to read the fumes temperature.
- `p_state`, this topic is used to read the flame power (%).
- `f_state`, this topic is used to read the fan speed (1-5).
- `cmd`, is the topic used to control the stove.

### Command currently supported by the Pellet Stove Control
|Command   |Description   | Write to |
|---|---|---|
|`ON`   |powers on the pellet stove  | RAM |
|`OFF`   | shutdown the pellet stove  | RAM |
|`Px`  |sets the power of the pellet stove. e.g. P2 sets the power @ 2 [0 to 5]  | EEPROM |
|`Fx`  |sets the fan speed of the pellet stove. e.g. F0 sets the fan speed to automatic [0 to 5] | EEPROM |
|`Txx`  |sets the target temperature of the pellet stove. e.g. T20 sets the target temperature to 20 °C [10-40]  | EEPROM |
|`E`  |resets any error on the stove   | RAM |

Since there is no other way to set flame power/fan speed/temperature rather than writing on the EEPROM, please pay attention to the final integration as EEPROM writes are limited (~10000).

## Home Assistant
For Home Assistant support, please contact me.
