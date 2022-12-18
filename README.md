# ESP32 OGN-Tracker (heltec lora v3)

OGN Tracker implementation on ESP32 devices.

![HELTEC LORA V3 mavlink](/images/heltec_v3.jpg)

This now only works for the Heltec V3 boards.

The [original](https://github.com/pjalocha/esp32-ogn-tracker) works with HELTEC and TTGO boards with sx1276 RF chip for 868/915MHz
The quickest board to run is the T-Beam from TTGO as it includes GPS, RF chip, battery circuit and holder, optionally as well a small OLED display. Yout to solder BMP280 or BME280 pressure/temperature/humidity sensor.

The initial code is written for and tested on HALTEC LoRa 32 module with sx1276 and 128x64 OLED display.
Most likely it can be easily ported to other ESP32 devices, as these are very flexible for the I/O assignement.
If you need to change the pins assigned to various periferials, see the top of the hal.cpp file.

## Recent development

Basic funcionality with Heltec V3 board. Lorawan not working

### LoRaWAN connectivity

not working yet

### IGC files recorded on the SD card

For OGN-Trackers with SD card connected, IGC files are recorded., as well as internal log files are copied over to the SD card in order not to be lost when newer files overwrite them.

### Wi-Fi Access Point

enabled by default with AP and HTTP

When compiled with WITH_AP WITH_WIFI and WITH_HTTP the OGN-Tracker creates a Wi-Fi access point when you can connect with the smartphone and access the status, configuration and log files stored in the flash memory.

Note the the ESP32 takes about 80mA more when the Wi-Fi AP is enabled thus total current consumed is about 200mA.

### Stratux-EU connectivity

not tested on V3

When compiled with WITH_STRATUX, WITH_WIFI and WITH_HTTP the OGN-Tracker can serve as source of GPS and pressure data (if pressure module present) to the Stratux. The OGN-Tracker connects to Wi-Fi access point created by Stratux Raspberry PI and send GPS and pressure data to port 30000.
Once the OGN-Tracker is connected to Stratux, it is possible to connect to its HTTP interface to access status, setup and log files.

## To compile the code and flash the ESP32 module: install the ESP-IDF

To compile and flash the ESP32 board you need to install the ESP-IDF v4.4
Start with:

```
cd
git clone -b v4.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
```
If you are doing this on a Raspberry PI and see error messages about *virtualenv* you may need the following:
```
sudo apt-get install libffi-dev
pip install --upgrade virtualenv==16.7.9
```

If you see errors about openssl/opensslv.h then you need:
```
sudo apt-get install libssl-dev
```

Then, in order to be able to *make* projects you need to run
```
source ~/esp-idf/export.sh
```

To get the OGN-Tracker source code from this github repository:
```
cd
git clone https://github.com/mlaiacker/esp32-ogn-tracker.git --recursive
cd esp32-ogn-tracker
idf.py build
```

To see the console output and thus to check if the ESP32 is alive
```
minicom
```
For minicom setup use 115200bps /dev/ttyUSB0 serial port and turn hardware and software handshake OFF. It is important, otherwise if you type something it won't be sent to the ESP32.
Press *Ctrl-C* to list the internal state and parameters. To change parameters, use $POGNS like this:
```
$POGNS,AcftType=1,Pilot=YourName
```

## Wiring the GPS

Wiring is fairly flexible, as the ESP32 can easily redefine the I/O signals. You need to avoid pins which are already used: the list normally comes with every ESP32 module. You can choose the pins after their placement on the board, for example such, that all wires are soldered on one side. Wiring for various modules has been chosen but if needed can easily be changed.

### HELTEC V3 boards without GPS

This here is configured for mavlink input on the GPS prt, connect your autopilot to this port and for PX4 configure mavlink output it for OSD.

```
#if defined(WITH_HELTEC_V3)
#define PIN_GPS_TXD GPIO_NUM_4 
#define PIN_GPS_RXD GPIO_NUM_5 
#define PIN_GPS_PPS GPIO_NUM_6 
#endif
```

## Wiring I2C (pressure sensors and/or OLED displays)

For devices with an OLED screen the I2C pins are already defined and you should follow these, as there is one common I2C bus in use.
For devices without we still use the same I/O pins.


## Console dialog and configuration

Use minicom and connect to /dev/ttyUSB0 (on Linux) for configuration set 115200bps and turn the hard- and soft-handshake OFF.
You shall see stream of NMEA sentences.
You can give the following commands:

Ctrl-C - lists internal state, internal log files and current parameter values.
Ctrl-L - list internal log files
Ctrl-V - hold the NMEA stream for 1 min
Ctrl-X - restarts the system

To set parameters send $POGNS with parameter name and value like
```
$POGNS,Pilot=John
```
the parameter value changes and all parameters are writen to internal flash thus they are preserved across system restart or repower.
To list all parameters with their values send Ctrl-C (software and hardware handshake must be OFF).

Note: more recently, the internal log files can be accessed through the WiFi Access Point and HTTP interface.
The drawback is higher power consumption due to WiFi.

## BT interface

OGN-Tracker can be connected via Bluetooth from Android devices. The BT port is like a serial port and carriers the same data as the USB serial port.

