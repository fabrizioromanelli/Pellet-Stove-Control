# Pellet Stove Control ![ESP8266](https://img.shields.io/badge/ESP-8266-000000.svg?longCache=true&style=flat&colorA=CCCC33) ![ESP32](https://img.shields.io/badge/ESP-32-000000.svg?longCache=true&style=flat&colorA=FF3500)

<a href="https://buymeacoffee.com/fabrizioroz" target="_blank">
  <img src="https://cdn.buymeacoffee.com/buttons/v2/default-yellow.png" alt="Buy Me A Coffee" width="200">
</a>

**I have prebuilt kits, ask me if you need any information!**

This project allows you to smart control any pellet stove controlled via a Micronova Controller N100.
The Arduino project has been compiled for an ESP8266 and successfully tested on a Micronova N100 Controller.
Any other board from Micronova could work, but this is left to the user/developer (probably something changes
on the memory mapping side).
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
- 1 x RJ11 male connector (if your stove has a female RJ11 connected to the serial interface).
- 1 x DB9 (DE-9) male connector (if your stove has an DB9 female connector).

## Circuit diagram
![Schematics](https://github.com/fabrizioromanelli/Pellet-Stove-Control/blob/master/PCB/Schematics.png)

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

## Micronova N100 serial port pinout

![Serial pinout for both DE-9 and RJ11 connectors](https://github.com/user-attachments/assets/bfa9301f-3aa0-4ab5-97e0-21d374b9199b)

The scheme shows the serial pinout from the Micronova N100 serial port with the connections for both the DE-9 connector and the RJ11 connector.

|Color|Function|
|---|---|
|`Blue`|`Serial`|
|`Red`|`+5V`|
|`Black`|`Ground`|

The fourth pin (the leftmost on the N100 board) is not used.

### Analysis of the pinout

There is a 4 pin connector (CN13 or SERIALE) with 5V, 20V, Serial, GND on the stove:

<img width="1794" height="747" alt="image" src="https://github.com/user-attachments/assets/c315ef4d-bf5c-4118-900a-2762f8363d59" />

The above schematics has to be checked on the stove itself as it can change from one integration to another.
If you have a multimeter, you can check it like this: put the COM of the voltmeter on the pin marked GND on my diagram and the red probe on the pin marked Serial, then move it on the pin 20V, then finally move it on the pin 5V. The results (in order) should be 5V, then 20V, then another 5V.
5V goes to 5V (from the stove), GND to GND (from the stove) and Serial to Serial (from the stove). The 20V pin is only used if you want to power the board from a buck converter via the 3.3V pin and not by USB.

## PCB project
The folder PCB contains the project for the Printed Circuit Board of the Pellet Stove Control.

Front PCB board:

<img width="2388" height="1665" alt="PCB_Front" src="https://github.com/user-attachments/assets/f6eb6efa-f8fe-48b1-9549-eaa83da76f4e" />

Back PCB board:

<img width="2388" height="1665" alt="PCB_Back" src="https://github.com/user-attachments/assets/4567a263-318d-4a51-a17c-b71abf29fffc" />

PCB Terminal plug:

<img width="372" height="292" alt="PCB_Terminal" src="https://github.com/user-attachments/assets/97412892-80aa-444d-95c5-fe9f6e5163a5" />

## Micronova board pinout
For the details related to the board pinout, refer to the manual. This project allows you to interface the stove with the serial port of the Micronova controller. Ask me in case of any doubts.

## Protocol for Micronova controller interface
The protocol for the Micronova controller has been reverse engineered and it is as stable as possible; there is a known problem with writing some values for controlling the stove (such as Flame Power and Fan Speed) that are mapped on the EEPROM (rather than on RAM), however this has been included in the project to let the user control all the stove parameters.

## Building the project
No MQTT parameter needs to be hardcoded in the sketch: everything is configured at first boot through the WiFiManager captive portal and then persisted on the board itself.

- Build the sketch with the Arduino IDE (no editing required)
- Upload the sketch on the board
- On first boot (or whenever the WiFi settings are reset), connect to the `PelletStove-Setup` WiFi network from your phone/PC (192.168.4.1)
- The browser should redirect to the configuration page where you can set up your WiFi credentials, together with the following MQTT parameters:
  - **MQTT server**, the IP or hostname of your MQTT broker
  - **MQTT port**, usually `1883`
  - **MQTT base topic**, the name of the topic you choose to publish on
  - **MQTT user**, if any
  - **MQTT password**, if any
- Once submitted, the board connects to your WiFi/MQTT broker and stores all these settings on the board's flash (LittleFS), so they are restored automatically on every subsequent reboot
- To change the settings later on, reset the WiFi credentials (e.g. via the WiFiManager reset routine) so the `PelletStove-Setup` portal comes up again, pre-filled with the previously saved values

Here follows some screenshots for the `PelletStove-Setup` portal:

<img width="1137" height="624" alt="Screenshot 2026-09-26 155310" src="https://github.com/user-attachments/assets/1e911fdc-ba94-4c32-8eab-7b61f8121fdf" />


WiFi network + MQTT parameters setup:


<img width="1143" height="939" alt="Screenshot 2026-09-26 155320" src="https://github.com/user-attachments/assets/62787a29-6cd3-4a07-b843-e6e7e3c31f74" />


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
An example of how the HA integration would look like in the Home Assistant Dashboard follows:

<img width="507" height="694" alt="stove_ui_english" src="https://github.com/user-attachments/assets/de362841-d9f4-4ab8-b2a0-c1b178b3bd08" />

