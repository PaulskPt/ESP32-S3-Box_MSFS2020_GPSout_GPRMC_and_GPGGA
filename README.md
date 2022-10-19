# Espressif 'ESP32-S3-Box_MSFS2020_GPSout_GPRMC_and_GPGGA'
 Receive, fliter and dispay MSFS2020 GPS data on the display of the 'esp-box'.


Display flown track and other GPS data to the display of the esp-box and use ground speed to control the displayed data

Software:
See 'Example'

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

To receive, filter and use certain elements of GPRMC GPS datagram data sent by an add-on called ```FSUIPC7``` to the ```Microsoft Flight Simulator 2020 (FS2020)```.
From the filtered GPRMC GPS type of datagram this project only uses the ```Track made good true``` and the ```groundspeed```. The track flown by the aircraft is displayed on the 4x20 serLCD, only when the groundspeed value exceeds a certain minimum value set in the micropython script. If the groundspeed is zero the aircraft is assumed to be halted or be parked. In that case the script will display ```Airplane stopped or parked```. When the groundspeed is > 0.2 and < 30 kts, the script will display ```Airplane is taxying```.  As soon as the groundspeed exceeds 30 kts the following data will be displayed onto the 4x20 serLCD:
```
- Latitude/Longitude;
- Altitude;
- Groundspeed
- Track (true) flown. 
```

```
Notes about the Sparkfun serLCD. 
- Contrast:
  The contrast of this device is quite depending on the applied Voltage. 
  My experience is that you have to experiment.
  See line 550 of the script:
  lcd.set_contrast(150)  # default value 120
- Pins are touch sensitive:
  Touching certain pins causes artifacts in the screen that can only be deleted by issuing a lcd.clear() 
  or resetting the device.


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

I used the Mu-editor app to save, edit and test the script file: ```code.py```.


Disclamer:
This project has been tested and working on a pc running MS Windows 11 Pro.

Other sources:
To read more about the GPS GPRMC datagram specification see: ```https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm```.

License: MIT (see LICENSE file)
