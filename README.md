# Espressif 'ESP32-S3-Box_MSFS2020_GPSout_GPRMC_and_GPGGA'
 Receive, fliter and dispay MSFS2020 GPS data on the display of the 'esp-box'.


Display flown track and other GPS data to the display of the esp-box and use ground speed to control the displayed data

Software:
See 'Examples'

Used hardware:

a) a Personal computer running Microsoft Windows 11 or a Microsoft XBox seriex X (not tested) on which are installed and running: 
    - (1) Microsoft Flight simulator 2020 (MSFS2020) (https://www.flightsimulator.com/);
    - (2) FSUIPC7 add-on app from Pete & John Dowson (http://www.fsuipc.com/);

b) Espressif ESP32-S3-Box (v 2.5): https://www.espressif.com/en/dev-board/esp32-s3-box-en or: https://www.adafruit.com/product/5290;

c) Adafruit CP2021N Friend - USB to Serial Converter (Product nr 5335, https://www.adafruit.com/product/5335) (signal levels are 3.3V);
   Other type of USB to RS232 serial converters I tested are:
   - Adafruit MCP2221A breakout - General Purpose USB to GPIO ADC I2C - Stemma QT / Qwiic (Product ID 4471 https://www.adafruit.com/product/4471).
   - Model YP-05. Attention: use a model that is able to provide or set for logic 3V3 level signals;

Flow of the GPS data:  PC MSFS2020 w FSUIPC7 > COMx > CP2102n TX/RX > esp-box U0RX (board.G44) / U0TX (board.G43).
```
Serial connection: CP2102N TX > esp-box U0RX
                   CP2102N RX > esp-box U0TX
                   CP2102N GND > esp-box GND
```
This project uses circuitpython.

Goals of this project:

To receive, filter and use certain elements of GPRMC and GPGGA GPS datagrams data sent by an add-on called ```FSUIPC7``` to the ```Microsoft Flight Simulator 2020 (FS2020)```.
From the filtered GPRMC GPS datagram message this project uses the airplane's position in ```Latitude``` and ```Longitude```, the ```groundspeed``` and the ```Track made good true```. From the filtered GPGGA GPS datagram message only the ```Altitude``` data is used. All the filtered GPS data is shown on the display of the esp-box. When the groundspeed value is > 0.2 and <= 30 kts, the airplane is assumed to be taxying. If the groundspeed is 0, during a short period, the airplane is assumed to be stopped or parked. The states: 'airplane is stopped or parked' and 'airplaine is taxying' are shown on the display. As soon as the groundspeed exceeds 30 kts the following data will be displayed onto the display of the esp-box.
```
- Latitude/Longitude;
- Altitude;
- Groundspeed
- Track (true) flown. 
```

```
+---------------------------------------+
| USB-UART-to-esp-box U0TX/U0RX pins:   |
+-----------------+---------------------+
|  USB-to-serial  |  esp-box            |
|  converter      |  pin:               |
|  pin:           |                     |
+-----------------+---------------------+
|- ```TX```pin    | to ```U0RX```pin    |
|- ```RX```pin    | to ```U0TX``` pin   |
+-----------------+---------------------+
```
NOTE: The baudrate is set to 4800 baud (inside the FSUIPC7 > GPSout > 1 (or > 2). There, also select the correct COM-port for MS Windows 11)

Data Indicator LED:
Many USB-to-Serial converters have a LED that signals the presence of data. The CP2102N and YP-5 listed under c) above have such a LED.
The script does blink the RGB LED on the piggy-back external board in color green at the moment valid GPS data has been received.

Two versions
============

Version 1:
==========
This is the original version;

Version 2:
==========
This version was only possible since @Danh (Discord > Adafruit) on October 27, 2022, solved a bug in CircuitPython V8.0.0-beta.3,
because it did crash the ESP32-S3-BOX as soon as the wifi.radio.connect() was called.
Version 2 has the following additions:
- date & time synchronization from Adafruit IO Time Service.
  The script connects automatically to the WiFi Access Point of your choice (set the ```ssid``` and ```password``` in file ```.env``` (do the same in file ```secrets.py``` for backup situations). When a WiFi connection has been established, the script, at intervals (set in function loop() by local variable: ```interval_t```. The value of this variable defaults to 600 seconds or 10 minutes), gets from ```Adafruit IO Time Service``` the date & time. Then the internal Realtime Clock (RTC) will be synchronized to the received date & time. Note that one needs to set the values for the keys ```aio_username``` and ```aio_key``` in file ```secrets.py```.
- Implementation of display touch. This is still in development. The idea is to 'jump' to a settings menu after a touch event has been recognized.
  In this moment are added the functions: ```clr_touch(), ck_touch() and menu_touch()```. At the start of the script, just behind the creation of the touch type object (tt), a call is being made to ```clr_touch()``` to empty touch events that could have been active at the start of the script. Then, in every iteration of ```loop()```, also a call has made to ```clr_touch()``` when the global flag ```tt_touched``` is True.

I used the Mu-editor app to save, edit and test the script file: ```code.py```. I also used VSCode to find bugs and to copy the list of variables and functions.


Disclamer:
This project has been tested and working on a pc running MS Windows 11 Pro.

Other sources:
To read more about the GPS GPRMC datagram specification see: ```https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm```.

License: MIT (see LICENSE file)
