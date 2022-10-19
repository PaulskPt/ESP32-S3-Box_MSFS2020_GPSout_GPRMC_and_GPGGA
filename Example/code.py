# SPDX-FileCopyrightText: 2022 Paulus Schulinck
#
# SPDX-License-Identifier: MIT
##############################
"""
    FOR CIRCUITPYTHON ONLY

    Copyright 2021 Paulus H.J. Schulinck (@paulsk on discord, CircuitPython, deepdiver member)
    Github: @PaulsPt
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
    and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all copies
    or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
    ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    End of license text

    (Using firmware: Adafruit CircuitPython 7.0.0-alpha.3-15-gbc014cecb on 2021-06-04; Raspberry Pi Pico with rp2040)

    This is the main python script for the project 'msfs2020_gps_rx',
    using a Raspberry Pi Pico dev board, mounted on a Seeed Grove Shield for Pi Pico
    Attached to it via I2C: a 20x4 character Hitachi 44780 LCD with piggy-back I2C expander.
    See the README.md and documentation folder for details about this project.

    Update 2021-10-10
    This version is modified to use it on an Adafruit (Unexpected Maker) FeatherS2 (= ESP32S2) board
    and a SparkFun SerLCD 20x4 RGB Qwwic display connected via a SparkFun logic level converter (5V / 3V3).
    (Using firmware: Adafruit CircuitPython 6.3.0 on 2021-06-01; FeatherS2 with ESP32S2)
    This version also has functions to read data details of:
    a) the operating system the board is running on. See function get_os_info();
    b) the (not so unique!) cpu id. See function get_cpu_id().  Update: 2022-10-17: Circuitpython has now: board.board_id.
    Both functions are called by the function my_board().

    Update 2022-07-04/05
    Changed platform to: an Unexpected Maker FeatherS2 (with Adafruit CircuitPython v.8.0.0-alpha.1).
    I2C via QT/Stemma connector to a 3.3 logic SparkFun SerLCD 16x4.
    I made a lot of changes in the split_types() function. For this I could delete the function bfr_fnd(). I made also changes to the ck_uart()
    function. In ck_uart() I added checks to see if the GPS messages of tyep $GPRMC and $GPGGA have there ID's and
    in the tail of the message is a '*' character.
    For UART I changed the use of board.UART() for busio.UART because the busio version gives me more settings flexibility:
    baudrate, timeout, receiver_buffer_size. I experienced a great decrease of failed/rejected msg receptions. Before these
    latest modifications, the message wait times were average 10.5 seconds with extremes of 13.6 seconds. Now, the average
    wait time for a messages is between 0.4 and 0.99 seconds, incidently extremes of 4.98 seconds. On 20 messages,
    these extremes happened 2 times. Comaring the average message wait time between the old and the new algorythm,
    gives an efficiency gain of a factor 20 (maximum) and a factor 10 (minimum).
    I added functionality to perform some reception time diagnostics. This functionality is controlled via the global flag 'use_diagnosics'.
    There is also a flag 'use_neopixel' which controls the use of the built-in RGB LED.
    This script has a 'gps_msgs' class which is used to store and retrieve the received gps msg data.
    The gps data comes from the FSUIPC7 add-on software for MSFS2020, via the Options menu, GPS Out..., set to the MS Windows 10/11 COM port
    of the USB-to-RS232 interface board and in FSUIPC7 GPS Out set for a baudrate of 4800 bps.

    Update 2022-10-17 16h22 utc+1
    This will be a version ported to an Espressif ESP32-S3-BOX.
    Now this is possible after my request to the maintainer of adafruit_tinyuf2 who, on my request, added the esp32-s3-box to tinyuf2/ports/espressif/boards.
    Next, in a real Ubuntu-Linux session I made a fresh clone of adafruit_tinyuf2. Next I was able to build and flash the tinyuf2 bootloader for the 'esp32s3_box'
    Finally I was able to copy the latest CircuitPython V8.0.0-beta.2 onto the esp32-s3-box.
    Now I start to try to port this script to the esp32-s3-box.
    2022-10-19 14h00: all code regarding 'neopixel' has been removed from this script for the esp32-s3-box
    because the box uses an external RGB-led module that can be attached to the box.

    ToDo:
    2022-10-19 14h00 utc+1: find out how to control (e.g. switch-off) the esp32-s3-box green pwr on/off led
    in the top of the box to the right of the WiFi ('air' mode) button.

    board(dir) has:
    ['__class__', '__name__', 'BOOT', 'CLK', 'CS', 'CTP_INT',
    'DISPLAY',
    'G10', 'G11', 'G12', 'G13', 'G14', 'G21', 'G38', 'G39', 'G40', 'G41', 'G42', 'G43', 'G44', 'G9',
    'I2S_ADC_SDOUT', 'I2S_CODEC_DSDIN', 'I2S_LRCK', 'I2S_MCLK', 'I2S_SCLK',
    'LCD_CS', 'LCD_CTRL', 'LCD_DC', 'LCD_MOSI', 'LCD_RST', 'LCD_SCK',
    'MISO', 'MOSI', 'MUTE_STATUS', 'PA_CTRL',
    'SCL', 'SCL2', 'SDA', 'SDA2',
    'U0RXD', 'U0TXD',
    'board_id']

    dir(board.DISPLAY) has:
    ['__class__', 'auto_refresh', 'brightness', 'bus', 'fill_row', 'height', 'refresh', 'root_group', 'rotation', 'show', 'width']

    dir(board.DISPLAY.root_group) has:
    ['__class__', 'append', 'index', 'insert', 'pop', 'remove', 'sort', 'hidden', 'scale', 'x', 'y']

    dir(busio) has:
    ['__class__', '__name__', 'I2C', 'SPI', 'UART']

    dir(displayio) has:
    ['__class__', '__name__', 'Bitmap', 'ColorConverter', 'Colorspace', 'Display', 'EPaperDisplay', 'FourWire',
    'Group',
    'I2CDisplay',
    'OnDiskBitmap', 'Palette', 'Shape', 'TileGrid',
    'release_displays']

"""

import board
import busio
import terminalio
import digitalio
import microcontroller
import sys, os, gc
from time import sleep, monotonic_ns
#import adafruit_dotstar as dotstar
#from displayio import Group
import displayio
from adafruit_display_text import label

# -----------------------+
# General debug flag     |
my_debug = False      #  |
# -----------------------+
# Other global flags     |
ctrl_c_flag = False   #  |
# -----------------------+
use_led3 = True       #  |
# -----------------------+
do_led3_test = False  #  |
# -----------------------+

my_os = None  # will hold a list containing O.S. info
my_machine = None
#               like fw version, machine name.
#               Data collected through function: get_os_info()

# +--------------------------------------+
# | Msg rx diagnostics                   |
# +--------------------------------------+
use_diagnosics = False
if use_diagnosics:
    diagnostics_iterations = 99 # Perform rx msg diagnostics for n times
    diagn_dict = {} # used for msg receive time diagnostics
else:
    diagn_dict = None # needs always to be defined. Used in 'globals' cmd in loop() and ck_uart()
    # See loop()
msg_rx_ok = 0
msg_rx_fail = 0

id = board.board_id

if board.board_id == 'espressif_esp32s3_box':

    i2c = busio.I2C(board.SCL, board.SDA)
    TX = board.G43  # U0TXD  # board.G43
    RX = board.G44  # U0RXD  # board.G44
else:
    i2c = None
    TX = None
    RX = None

# +--------------------------------------+
# | Definitions for all LEDs             |
# +--------------------------------------+
led_colors_dict = {
    'green' : 0,
    'red' : 1,
    'blue' : 2,
    'white' : 3,
    'off' : 4 }

lcd_bl_colors = ["black", "red", "orange", "yellow", "green", "blue", "indigo", "violet", "grey", "white"]

brill = 50

colorN = [(0,  0,     0),     # black is off
  (brill,       0,     0),     # bright red
  (0xFF8C00,    0,     0),     # orange
  (brill,       brill, 0),     # bright yellow
  (0,           brill, 0),     # bright green
  (0,           0,     brill), # bright blue
  (0x4B0082,    0,     0),     # indigo
  (0xA020F0,    0,     0),     # violet
  (0x808080,    0,     0),     # grey
  (brill,       brill, brill)] # white

# Create a colour wheel index int
color_index = 0
lcd_color_index = 0
lcd_clr_chg_cnt = 0

# Create a colour wheel index int
color_index = 0
lcd_color_index = 0
lcd_clr_chg_cnt = 0

if use_led3:
    # +--------------------------------------+
    # | Definitions for external 3-color LED |
    # +--------------------------------------+
    LED_B = board.G39
    LED_G = board.G40
    LED_R = board.G41
    led3_red   = digitalio.DigitalInOut(LED_R)
    led3_green = digitalio.DigitalInOut(LED_G)
    led3_blue  = digitalio.DigitalInOut(LED_B)
    led3_red.direction   = digitalio.Direction.OUTPUT
    led3_green.direction = digitalio.Direction.OUTPUT
    led3_blue.direction  = digitalio.Direction.OUTPUT
    HIGH = 1
    LOW  = 0

    # Copied from dropbox/paul/hardware/adafruit/Feather/FeatherS2_nr2/GPRMC_via_serial/code.py line 195 ..
    color_t_arr = ["GREEN  ", "RED    ", "BLUE   ", "RAINBOW", "OFF"]
    c_brill = 20
    bril_dim = 5
    #                GREEN            RED              BLUE              WHITE                          OFF
    color_c_arr = [(0, c_brill, 0), (c_brill, 0, 0), (0, 0, c_brill), (bril_dim, bril_dim, bril_dim), (0, 0, 0)]

    led_interval = 1000
    led_state = HIGH  # idem. When HIGH the LED is OFF
else:
    HIGH = None
    LOW = None
    LED_R = None
    LED_G = None
    LED_B = None


biLdIsOn = False # Flag for the FeatherS2 built-in blue led

# +-----------------------------------------------+
# | Create an instance of the UART object class   |
# +-----------------------------------------------+
if my_debug:
    print()
    print(f"globals: TX= {TX}, RX= {RX}")
uart = busio.UART(TX, RX, baudrate=4800, timeout=0, receiver_buffer_size=151)  # board.RX, board.TX)
#uart = board.UART()

# release any currently configured displays
#displayio.release_displays()

display = board.DISPLAY
pge1_group = None  # group to hold our texts
pge1_lbl = []      # list of labels
max_text_len = 25
font =  terminalio.FONT
color = 0x00FF00
scale = 2
WIDTH = display.width   # get the display width (240)
HEIGHT = display.height  # get the display height (320)
display.rotation = 0  # set the rotation
rotation = display.rotation  # get the current rotation
dx = 0  #100
dy = 10
disp_pos_dict = { 0: (dx+290, dy),
    1: (dx, dy+40),
    2: (dx, dy+80),
    3: (dx, dy+120),
    4: (dx, dy+160),
    5: (dx, dy+200)
}

if sys.version_info > (3,):
    long = int


nRMC = None
nGGA = None

_id = 0
_lat = 1
_latdir = 2
_lon = 3
_londir = 4
_gs = 5
_crs = 6
_alt = 7

class gps_msgs:
    def __init__(self):
        self.gps = ["",  "",  "",  "",  "", "", "", ""]

    def write(self, s):
        tp = isinstance(s,list)
        if tp == True:
            self.gps[_id] = s[_id] # ID
            self.gps[_lat] = s[_lat] # Lat
            self.gps[_latdir] = s[_latdir] # LadID N/S
            self.gps[_lon] = s[_lon] # Lon
            self.gps[_londir] = s[_londir] # LonID E/W
            self.gps[_gs] = s[_gs] # GS
            self.gps[_crs] = s[_crs] # CRS
            self.gps[_alt] = s[_alt] # Alt

    def read(self, n):
        tp = isinstance(n, type(None))
        if tp == True:
            n = 0
        if n >= 0 and n <= 7:
            return self.gps[n]
        else:
            return self.gps

    def clean(self):
            self.gps[0] = ""
            self.gps[1] = ""
            self.gps[2] = ""
            self.gps[3] = ""
            self.gps[4] = ""
            self.gps[5] = ""
            self.gps[6] = ""
            self.gps[7] = ""

#encoding = 'utf-8'
lcd_maxrows = 4
lcd_rowlen = 20
lp_cnt = 0
max_lp_cnt = 99
startup = -1
loop_time = 0
t_elapsed = 0
msg_nr = 0

# next four defs copied from:
# I:\pico\paul_projects\pico\circuitpython\msfs2020_gps_rx_picolipo\2021-09-03_16h49_ver
lac_Stopped = True
lacStopMsgShown = False
acStopInitMonot = 0
acStopInterval = 6000 # mSec

# Buffers
rx_buffer_len = 160
rx_buffer = bytearray(rx_buffer_len * b'\x00')

# Classes
my_msgs = gps_msgs()


"""
   get_os_info() -> Bool
        @brief
        Get and extract the name and version of the CircuitPython firmware installed,
        e.g. 'release='6.3.0' and 'version='6.3.0 on 2021-06-01'.
        Get also the name of the 'machine', e.g.: FeatherS2 with ESP32S2'
    Parameters: None
    Return: Bool

"""
def get_os_info():
    global my_os, my_debug, my_machine
    my_fw_itms = ("sysname", "nodename", "release", "version", "machine")
    n = os.uname()
    if n:
        my_os = n
        le = len(my_os)
        my_machine = my_os[le-1]
        if my_debug:
            print("get_os_info(): my_machine=", my_machine)
        if my_debug:
            for i in range(le):  # sysname, nodename, release, version and machine
            # e.g.: (sysname='esp32s2', nodename='esp32s2', release='6.3.0',
            # version='6.3.0 on 2021-06-01', machine='FeatherS2 with ESP32S2')
                print("{}: \'{}\'.".format(my_fw_itms[i], my_os[i]), end='\n')
        return True
    else:
        if my_debug:
            print("get_os_info(): n: {}, type(n): {}".format(n, type(n)), end='\n')
        return False

"""
   my_board() -> None
        @brief
        Function prints global variables my_os and id
        if global my_debug is True and if either of these variables contain values
        Called by setup()
    Parameters: None
    Return: None

    Live result:
    'OS: (sysname='ESP32S3', nodename='ESP32S3', release='8.0.0', version='8.0.0-beta.2 on 2022-10-14', machine='ESP32-S3-Box-2.5 with ESP32S3').'
"""
def my_board():
    global my_os, id
    if my_debug:
        if my_os:
            print("OS: {}.".format(my_os), end='\n')
        if id:
            print("CPU ID: {}.".format(id), end='\n')

"""
    create_group(void) -> None
        @brief
        This function is called by setup().
        It creates a displayio group and the labels within it

        Parameters: None

        Return: None
"""
def create_group():
    global pge1_group, pge1_lbl
    TAG = "create_group(): "
    # make 1 page of content
    pge1_group = displayio.Group()

    # labels
    s = ' '*max_text_len
    pge1_lbl = []
    le = len(disp_pos_dict)
    if my_debug:
        print(TAG+f"going to create {le} page1 labels")
    for _ in range(le):
        dx = disp_pos_dict[_][0]
        dy = disp_pos_dict[_][1]
        pge1_lbl.append( label.Label(
            font=font,
            scale=scale,
            text=s,
            anchor_point=(0, 0),
            anchored_position=(dx, dy),
        ))
        if my_debug:
            print(TAG+f"pge1_lbl[{_}] at anchored position: x={dx}, y={dy}")

    if my_debug:
        print(TAG+f"pge1_group= {pge1_group}")
        print(TAG+f"len(pge1_group)= {len(pge1_group)}")
        print(TAG+f"len(pge1_lbl)= {len(pge1_lbl)}")
    for _ in range(len(pge1_lbl)):
        pge1_group.append(pge1_lbl[_])
    if my_debug:
        print(TAG+f"len(pge1_group)= {len(pge1_group)}")

    return pge1_lbl
"""
    clear_labels(void) -> None
        @brief
        This function clears the texts of all labels

        Parameters: None

        Return: None
"""
def clear_labels():
    global pge1_group, pge1_lbl, max_text_len
    s = ' '*max_text_len
    for _ in range(len(pge1_lbl)):
        pge1_lbl[_].text = s
    display.show(pge1_group)

"""
    setup(void) -> None
        @brief
        This function is called by main().
        It sets starting parameters for the lcd:
        - no system messages (to the display);
        - no cursor;
        - (if cursor) do not blink the cursor.
        It also resets the uart input buffer

        Parameters: None

        Return: None
"""
def setup():
    global lcd, uart, degreesChar
    TAG = "setup(): "
    if my_debug:
        print(TAG+"...")
    # Here, is necessary, initialization of the esp32-s3-box display
    #create_group()

    if led_state == LOW: # Switch off the RGB LED
        led_toggle()
        #neopix_led_off()

    #lcd_chr_test()  # print all the characters in the lcd rom

    if uart:
        uart.reset_input_buffer()  # Clear the rx buffer

    sleep(1)  # <--------------- DELAY ---------------

    get_os_info()  # Collect O.S. info. Put in global variable my_os and (partly) in my_machine


"""
    loop(void) -> boolean
        @brief
        This functions is the backbone of this script. It is called by main().
        It makes calls to various important functions in this script.
        It also will stop execution if the user pressed the Ctrl-C key combo (keyboard interrupt).
positive
        Parameters: None

        Return: boolean
"""
def loop():
    global startup, lp_cnt, led_state, biLdIsOn, lcd, msg_nr, ctrl_c_flag, diagn_dict, pge1_group, pge1_lbl, scale, font, color, msg_rx_ok, msg_rx_fail
    TAG = "loop(): "
    if my_debug:
        print(TAG+"...")
    lRetval = True  # assume positive
    chrs_rcvd = 0
    lstop = False

    # lcd.clear()

    if my_machine:
        print(TAG+"my_machine= \"{}\"".format(my_machine))
        n1 = my_machine.find("ESP32S")
        n2 = my_machine.find("with")
        if n1 > 0 and n2 >=0:
            s3 = my_machine[:n2]+my_machine[n2:n2+1]+" "+my_machine[n2+5:]
        else:
            s3 = my_machine[:19]  # Not more than 20 characters

        #lcd.write(s3)
        #print(TAG+"my_machine (cut)= \"{}\"".format(s))
    else:
        s3 = sys.platform

    s0 = ("MSFS 2020      ", "GPRMC/GPGGA data RX", "Platform ", s3)
    # pge1_lbl[2..5]
    le = len(s0)
    for _ in range(le):
        pge1_lbl[_+2].text = s0[_]
    display.show(pge1_group)

    sleep(5)

    # Clear only texts of pge1_lbl[4..5]
    s = ' '*max_text_len
    for _ in range(2):
        pge1_lbl[_+4].text = s
    display.show(pge1_group)

    print()
    print("MSFS2020 GPS GPRMC data reception decoder script")
    print("(https://github.com/PaulskPt/ESP32-S3-Box_MSFS2020_GPSout_GPRMC_and_GPGGA")
    print("\nNumber of loops in this run: {}".format(max_lp_cnt))
    chrs_rcvd = 0
    print("........................", end="\n")

    while True:
        try:
            lp_cnt += 1
            # lcd.clear()
            print("\nStart of loop {}".format(lp_cnt), end="\n")

            if led_state == LOW: # Switch off the dotstar RGB LED
                led_toggle()

            if startup == -1:
                uart.reset_input_buffer()
                pge1_lbl[5].text = "About to receive..."
                display.show(pge1_group)
            wait_cnt = 0
            chrs_rcvd = ck_uart()
            print(TAG+"characters rcvd: ", chrs_rcvd)
            if chrs_rcvd > 0:
                lResult = split_types()
                print(TAG+"split_types() result = {}".format(lResult))

                if lResult == True:
                    msg_rx_ok += 1
                    if not is_ac_stopped():
                        msg_nr += 1
                        blink_led3(led_colors_dict['green']) # Switch the external 3-color LED GREEN
                        print(TAG+"handling msg nr: {:02d}".format(msg_nr))
                        if use_diagnosics:
                            if msg_nr in diagn_dict:
                                diagn_dict[msg_nr][1] = 1 if lResult else 0
                            else:
                                print(TAG+"msg_nr {} not found in diagn_dict".format(msg_nr))
                        lcd_pr_msgs()
                        gc.collect()
                        print(TAG+f"gc.mem_free()= {gc.mem_free()}")
                        chrs_rcvd = 0
                        #sleep(0.1)
                        if msg_nr >= max_lp_cnt:
                            pass
                        if lstop == True:
                            if biLdIsOn:
                                #led.value = 0
                                #feathers2.led_set(0)
                                biLdIsOn = False
                        else:
                            if startup == -1:
                                print("Waiting for serial com line to become available...")
                                startup = 0
                        print("End of loop {:2d}".format(lp_cnt), end="\n")
                        print("........................", end="\n")
                else:
                    msg_rx_fail += 1
            if msg_nr >= max_lp_cnt:
                pct_ok = msg_rx_ok / lp_cnt
                pct_fail = msg_rx_fail / lp_cnt
                print(TAG+f"Msgs rx OK= {msg_rx_ok}, rx Fail= {msg_rx_fail}, pct OK= {pct_ok} pct fail= {pct_fail}")
                msg_nr = 0
                lp_cnt = 0
                msg_rx_ok = 0
                msg_rx_fail = 0
            if lstop == True:
                break
            if use_diagnosics:
                # Prepare and print to REPL a rx msgs diagnostics report
                if len(diagn_dict) >= diagnostics_iterations:
                    avg_time = 0
                    avg_split = 0
                    v = s = 0
                    le = len(diagn_dict)
                    for k, v in sorted(diagn_dict.items()):
                        if isinstance(diagn_dict[k], dict):
                            le2 = len(diagn_dict[k])
                            if le2 == 1:
                                v = diagn_dict[k][0]
                            if le2 == 2:
                                v = diagn_dict[k][0]
                                s = diagn_dict[k][1] # split result
                                avg_split += s
                        if isinstance(diagn_dict[k], float):
                            v = diagn_dict[k]
                        avg_time += v
                        print("msg nr: {:2d}, wait time for msg: {:7.4f}. Split result: {}".format(k, v, s))
                    print("Average wait time for msg: {:>5.2f}. Average split result: {:>5.2f}".format(avg_time/le, float(avg_split/le)))
                    diagn_dict = {} # cleanup the diagnostics dict
        except KeyboardInterrupt:
            ctrl_c_flag = True
            s0 = ("\'Ctrl+C\' pressed.", "Going to quit...")
            print(s0[0], end='')
            print(s0[1], end='\n')
            # lcd.clear()
            pge1_lbl[4].text = s0[0]
            pge1_lbl[5].text = s0[1]
            display.show(pge1_group)
            sleep(5)
            lRetval = False
            break
    #sleep(0.5)
    return lRetval

"""
    ck_uart(void) -> int (nr_bytes)
        @brief
        This functions reads the incoming data via the uart.
        The received data is put into rx_buffer.
        If data received this function will return the number of bytes received.
        The function will loop if the value(s) of the received data is zero.
        Parameters: None

        Return: int
"""
def ck_uart():
    global rx_buffer, msg_nr, loop_time, diagn_dict, nRMC, nGGA
    TAG = 'ck_uart(): '
    nr_bytes = i = 0
    delay_ms = 0.3
    if use_diagnosics:
        rx_wait_start = monotonic_ns()
    while True:
        nr_bytes = uart.readinto(rx_buffer)
        #for i in range(5):
        #    sleep(delay_ms)
        loop_time = monotonic_ns()
        if nr_bytes is None:
            if my_debug:
                print(TAG+"nr_bytes is None")
            sleep(delay_ms)
            continue
        if not nr_bytes:
            if my_debug:
                print(TAG+"nr_bytes=", nr_bytes)
            sleep(delay_ms)
            continue
        if nr_bytes > 1:
            if not my_debug:
                print(TAG+"nr of bytes= ", nr_bytes)
                print(TAG+"rcvd data: {}".format(rx_buffer),end="\n")
            if nr_bytes < 150: # Try to cut short checking. A full/OK msg has 153 characters
                sleep(delay_ms)
                continue
            # Check for '*' in tail. If present, the message is very probably complete.
            tail_part = -17
            n1 = rx_buffer[tail_part:].find('*') # find '*' in tail 10 characters. If not found, loop
            #n1 = rx_buffer.find('*')
            # print(TAG+f"n1 = {n1}")
            if n1 < 0: # no '*' in tail
                sleep(delay_ms)
                continue
            else:
                n2 = rx_buffer.find('$GPRMC')
                # print(TAG+f"n2 = {n2}")
                if n2 < 0:
                    sleep(delay_ms)
                    continue
                else:
                    nRMC = n2

                n3 = rx_buffer.find('$GPGGA')
                # print(TAG+f"n3 = {n3}")
                if n3 < 0:
                    sleep(delay_ms)
                    continue
                else:
                    nGGA = n3
                n4 = int(nr_bytes*0.75) # get 3/4 of length of characters received
                # print(TAG+f"n4 = {n4}")
                if not my_debug:
                    #print(TAG+"n1 = {}, n2 = {}, n3= {}, n4 ={}".format(n1, n2, n3, n4))
                    print(TAG+"* at {}, $GPRMC at {}, $GPGGA at {}, n4 ={}".format(n1+tail_part+1, n2, n3, n4))
                if n2 > n4: # check if $GPRMC occurs at more than 3/4 of the characters received
                    sleep(delay_ms)
                    continue
                else:
                    # print(TAG+"returning with {} nr of bytes".format(nr_bytes))
                    if use_diagnosics:
                        rx_wait_stop = monotonic_ns()
                        rx_wait_duration = float((rx_wait_stop - rx_wait_start) / 1000000000) # convert nSec to mSec
                        diagn_dict[msg_nr+1] = {0: rx_wait_duration, 1: -1} # add a key/value pair for diagnostics
                        print(TAG+"it took {:6.2f} seconds for a complete msg ($GPRMC & $GPGGA) to be received".format(rx_wait_duration))
                    #return nr_bytes
                    break
        elif nr_bytes == 1:
            if rx_buffer[0] == b'\x00':
                sleep(delay_ms)
                i += 1
                if i % 1000 == 0:
                    print("Waiting for uart line to become ready")
        else:
            empty_buffer()
            sleep(delay_ms)
            continue
    return nr_bytes

"""
   find_all(c) -> dict
   dict consists of an integer as key
   and an integer as value

"""
def find_all(c):
    #global rx_buffer
    f_dict = {}
    if c is None:
        return 0
    if type(c) is int:
        c2 = chr(c)
    elif type(c) is str:
        c2 = c
    t_rx_buffer = rx_buffer.decode()
    le = len(t_rx_buffer)
    if le > 0:
        ret = t_rx_buffer.count(c2)
        o_cnt = 0
        for i in range(le):
            if t_rx_buffer[i] == c2:
                f_dict[o_cnt] = i
                o_cnt += 1
    return f_dict

"""
    split_types(void) -> boolean
        @brief
        This functions searches the rx_buffer various times to find certain patterns
        that are tipical for the GPRMC and GPGGA GPS datagrams.
        If found, the data will be saved in the my_msgs class
        Parameters: None

        Return: boolean

"""
def split_types():
    global rx_buffer, my_msgs, nRMC, nGGA
    TAG = "split_types(): "
    lResult = True
    sRMC = sGGA = None
    rmc_s = gga_s = ""
    nr_RMC_items = nr_GGA_items = 0
    le_b4 = le_aft = 0
    nRMC_end = nGGA_end = 0
    lGPRMC_go = lGPGGA_go = False
    msg_lst = []
    rmc_lst = []
    gga_lst = []
    rmc_msg_lst = []
    gga_msg_lst = []
    le_dict = 0

    if my_debug:
        print(TAG+"entry...contents rx_buffer=", rx_buffer)
    t_rx_buffer = rx_buffer.decode()
    le_t_rx = len(t_rx_buffer)

    f_dict = find_all(10)
    if f_dict:
        le_dict = len(f_dict)
        if my_debug:
            print(TAG+"f_dict= {}, length dict = {}".format(f_dict, le_dict))
    if le_dict < 2:
        print(TAG+"exiting... f_dict length < 2")
        return False

    if nRMC is None: # in ck_uart() already has been done a find for $GPRMC and if found set nRMC
        nRMC = t_rx_buffer.find('$GPRMC')
    if nGGA is None: # idem for $GPGGA and nGGA
        nGGA = t_rx_buffer.find('$GPGGA')

    if le_dict >= 1 and (nRMC < 0 or nRMC > f_dict[le_dict-1]):
        print(TAG+"exiting.. nRMC = {}, f_dict[le_dict-1] = {}".format(nRMC, f_dict[le_dict-1]))
        return False  # $GPRMC not in first or not in first two msgs

    if nRMC >= 0:
        rmc_s = t_rx_buffer[nRMC:nRMC+6]
    if nGGA >= 0:
        gga_s = t_rx_buffer[nGGA:nGGA+6]

    if my_debug:
        print(TAG+"nRMC=", nRMC)
        print(TAG+"nGGA=", nGGA)

    if nRMC >= 0 and nGGA >= 0:
        if nRMC < nGGA:
            if nRMC >= 0:
                if le_dict == 1 or le_dict == 2:
                    sRMC_end = f_dict[0]+1
                    sRMC = t_rx_buffer[nRMC:f_dict[0]+1] # copy $GPRMC...\r\n'
                    if my_debug:
                        print(TAG+"sRMC = t_rx_buffer[{}:{}]".format(nRMC, sRMC_end))
                if le_dict == 3:
                    sRMC = t_rx_buffer[nRMC:f_dict[1]+1] # idem
            if nGGA > 0:
                if le_dict == 2:
                    if nGGA > f_dict[1]:
                        nGGA_end = le_t_rx
                    else:
                        nGGA_end = f_dict[1]+1
                    sGGA = t_rx_buffer[nGGA:nGGA_end] # copy $GPGGA...\r\n'
                    if my_debug:
                        print(TAG+"sGGA = t_rx_buffer[{}:{}]".format(nGGA, nGGA_end))
                if le_dict == 3:
                    sGGA = t_rx_buffer[nGGA:f_dict[2]+1] # idem
        elif nGGA < nRMC:
            if nGGA >= 0:
                if le_dict == 1 or le_dict == 2:
                    nGGA_end = f_dict[0]+1
                    sGGA = t_rx_buffer[nGGA:nGGA_end] # copy $GPGGA...\r\n'
                    if my_debug:
                        print(TAG+"sGGA = t_rx_buffer[{}:{}]".format(nGGA, nGGA_end))
                if le_dict == 3:
                    sGGA = t_rx_buffer[nGGA:f_dict[1]+1] # idem
            if nRMC > 0:
                if le_dict == 2:
                    if nRMC > f_dict[1]:
                        nRMC_end = le_t_rx
                    else:
                        nRMC_end = f_dict[1]+1
                    sRMC = t_rx_buffer[nRMC:nRMC_end] # copy $GPRMC...\r\n'
                    if my_debug:
                        print(TAG+"sRMC = t_rx_buffer[{}:{}]".format(nRMC, nRMC_end))
                if le_dict == 3:
                    sRMC = t_rx_buffer[nRMC:f_dict[2]+1] # idem

    if my_debug:
        print(TAG+"nRMC= ", nRMC)
        print(TAG+"rmc_s=", rmc_s)
        print(TAG+"sRMC=", sRMC)

        print(TAG+"nGGA= ", nGGA)
        print(TAG+"gga_s=", gga_s)
        print(TAG+"sGGA=", sGGA)

    lIsStr = isinstance(sRMC,str)
    if lIsStr:
        rmc_msg_lst = sRMC.split(",")
        nr_RMC_items = len(rmc_msg_lst)
        if nr_RMC_items == 12:
            lGPRMC_go = True

    lIsStr = isinstance(sGGA,str)
    if lIsStr:
        nGGA_ID = sGGA.find("$")
        if nGGA_ID > 0:
            pass
        else:
            gga_msg_lst = sGGA.split(",")
            nr_GGA_items = len(gga_msg_lst)
            if nr_GGA_items == 15:
                t_alt = float(gga_msg_lst[9])
                p = isinstance(t_alt,float)
                if p == True:
                    #                                  alt
                    t_alt = round(int(float(gga_msg_lst[9])) * 3.2808)
                else:
                    t_alt = 0
                lGPGGA_go = True

        if lGPRMC_go == True:
            #                      id             lat             latdir          lon           londir            gs        track true
            rmc_lst = [rmc_msg_lst[0], rmc_msg_lst[3], rmc_msg_lst[4], rmc_msg_lst[5], rmc_msg_lst[6], rmc_msg_lst[7], rmc_msg_lst[8]]
            if my_debug:
                print(TAG+"rmc_lst=", rmc_lst)
            if lGPGGA_go == True:
                rmc_lst.append(str(t_alt))
            else:
                rmc_lst.append("0")
            my_msgs.write(rmc_lst)
        if my_debug:
            print(TAG+"cross-check: my_msgs class data contents: {}".format(my_msgs.read(9)), end="\n")

    empty_buffer() # not emptying the buffer here causes an error in lcd_pr_msgs() !!!
    if lGPRMC_go == False and lGPGGA_go == False:
        lResult = False

    return lResult

"""
    wheel(int) -> r, g, b
        @brief
        This functions returns r, g, b values
        after calculation with parameter wheel_pos value

        Parameters: None

        Return: None
"""
def wheel(wheel_pos):
    """Color wheel to allow for cycling through the rainbow of RGB colors."""
    wheel_pos = wheel_pos % 255

    if wheel_pos < 85:
        return 255 - wheel_pos * 3, 0, wheel_pos * 3
    elif wheel_pos < 170:
        wheel_pos -= 85
        return 0, wheel_pos * 3, 255 - wheel_pos * 3
    else:
        wheel_pos -= 170
        return wheel_pos * 3, 255 - wheel_pos * 3, 0

"""
    led_BI_toggle(void) -> void
        @brief
        This functions toggles the builtin blue LED
        THIS FUNCTION IS USED FOR THE MOMENT (2021-10-11)

        Parameters: None

        Return: None
"""
def led_BI_toggle():
    pass

    """
    global led, biLdIsOn

    if biLdIsOn:
        #led.value = 0
        feathers2.led_set(0)
        biLdIsOn = False
    else:
        #led.value = 1
        feathers2.led_set(1)
        biLdIsOn = True
    """

"""
    led_toggle(void) -> void
        @brief
        This functions toggles the external 3-color LED

        Parameters: None

        Return: None
"""
def led_toggle():
    global led_state, HIGH, LOW
    # uses global variable led_state
    led_chrs_rcvd = 0 # Before this was a global defined variable.
    brightness = 0.1
    if led_state == HIGH:
        led_state = LOW  # a=!a
    elif led_state == LOW:
        led_state = HIGH  # a=!a

    if led_state == LOW:
        # Get the R,G,B values of the next colour
        blink_led3(led_colors_dict['off'])  # Switch the external 3-color LED off
    else:
        # Get the R,G,B values of the next colour
        r,g,b = wheel( color_index )

        # Increase the wheel index
        color_index += 2
        # If the index == 255, loop it
        if color_index == 255:
            color_index = 0
        # Invert the internal LED state every half colour cycle
        led_blink()
        # Sleep for 15ms so the colour cycle isn't too fast
        sleep(0.015)
        # [0] = color_c_arr[led_colors_dict['green']]  # Set the color of the builtin LED to GREEN
        # .show()
        blink_led3(led_colors_dict['green']) # Switch the external 3-color LED GREEN

"""
    led3_test() -> void
        @brief
        This function tests a series of colors of the the external 3-color LED

        Parameters: None

        Return: None
"""
def led3_test():
    global pge1_lbl
    TAG= "led3_test(): "
    lbl_nr = 5
    clear_labels()
    pge1_lbl[3].text = 'External LED color test:'
    for _ in range(3):
        print(TAG+f"test nr: {_+1}")
        for i in range(len(color_t_arr)):
            s = "showing led color: {:s}".format(color_t_arr[i])
            print(s)
            pge1_lbl[lbl_nr].text = s
            clr = color_c_arr[i]
            blink_led3(i)
            sleep(2)

    clear_labels()
    blink_led3("off")  # switch off the external RGB led
    print(TAG+"end of test")

"""
    blink_led3(i) -> void
        @brief
        This function sets the external 3-color LED to a color

        Parameters: i
            Parameter i is an index to the color

        Return: None
"""
def blink_led3(i):
    if i == led_colors_dict['green']:  # --- Green LED on ---
        led3_green.value = True
        led3_red.value = False
        led3_blue.value = False
    elif i == led_colors_dict['blue']:  # --- Red LED on ---
        led3_green.value = False
        led3_red.value = True
        led3_blue.value = False
    elif i == led_colors_dict['red']:  # --- Blue LED on ---
        led3_green.value = False
        led3_red.value = False
        led3_blue.value = True
    elif i == led_colors_dict['white']:  # --- All three LEDs on (= White) ---
        led3_green.value = True
        led3_red.value = True
        led3_blue.value = True
    elif i == led_colors_dict['off']:  # --- All three LEDs off (= Black) ---
        led3_green.value = False
        led3_red.value = False
        led3_blue.value = False
    else:
        return  # Do nothing if value out of range

"""
    led_blink(void) -> void
    Switches the built-in led (green led on top of the esp32-s3-box off
"""
def led_blink():
    pass

"""
 Function copied from: I:\pico\paul_projects\pico\circuitpython\msfs2020_gps_rx_picolipo\2021-09-03_16h49_ver
"""
def is_ac_stopped():
    global lac_Stopped, lacStopMsgShown, acStopInitMonot, acStopInterval, pge1_group, pge1_lbl
    TAG = "is_ac_stopped(): "
    s = "Aircraft is stopped or parked"
    #gs = 5
    t_elapsed = 0
    lelapsed = False
    t_gs = my_msgs.read(_gs)
    le = len(t_gs)
    if not my_debug:
        print(TAG,"value of gs = {}, len(gs) = {}".format(t_gs, le), end='\n')
    if le == 0: # check for t_gs == "". This happened sometimes!
        v_gs = 0
    else:
        v_gs = round(int(float(t_gs)))
    if not my_debug:
        print(TAG,"value of v_gs = {}".format(v_gs), end='\n')
    if v_gs == 0:
        lac_Stopped = True
    else:
        lac_Stopped = False
        lacStopMsgShown = False
        acStopInitMonot = 0
        return lac_Stopped

    if lac_Stopped == True:
        currMonot = monotonic_ns()
        if acStopInitMonot == 0:
            acStopInitMonot = currMonot

        t_elapsed = (((currMonot - acStopInitMonot) + 500000)// 1000000)
        if my_debug:
            print(TAG,"t_elapsed:{}. acStopInterval: {}".format(t_elapsed, acStopInterval), end='\n')

        if t_elapsed >= acStopInterval:
            lelapsed = True

        if not my_debug:
            print(TAG,"lelapsed:{}. lacStopMsgShown: {}".format(lelapsed, lacStopMsgShown), end='\n')
        if lelapsed:
            if lacStopMsgShown == False:
                # lcd.clear()  # It takes about ten seconds before this message is shown after aircraft is stopped
                pge1_lbl[2].text = 'Airplane is stopped'
                pge1_lbl[3].text = 'or parked'
                display.show(pge1_group)
                lacStopMsgShown = True
            print(s, end = '\n') # Alway print to REPL (it does almost immediately)
    return lac_Stopped


def empty_buffer():
    global rx_buffer, rx_buffer_len
    rx_buffer = bytearray(rx_buffer_len * b'\x00')


def lcd_pr_msgs():
    global startup, loop_time, t_elapsed, msg_nr, my_msgs, lcd_maxrows, lacStopMsgShown, lac_Stopped, pge1_group, pge1_lbl
    TAG = "lcd_pr_msgs(): "
    dp = 0
    msg_itm = 0
    lcd_vpos = 0
    s = s0 = ""

    lat   =  1
    latID =  2
    lon   =  3
    lonID =  4
    gs    =  5
    crs   =  6
    alt   =  7

    if id == 'espressif_esp32s3_box':
        degs = chr(0x5e)  # 0x2a)  # = '*'
    else:
        degs = chr(0xdf)
    #lac_Stopped = False
    #lacStopMsgShown = False
    if startup == -1 or lacStopMsgShown:
        pass
        # lcd.clear()
    s = "{:0>2d}".format(msg_nr)
    x_tmp = pge1_lbl[0].x  # save x
    #pge1_lbl[0].x = 18
    #pge1_lbl[0].y = 0
    pge1_lbl[0].text = s
    pge1_lbl[5].text = ' '*max_text_len  # clear status line
    display.show(pge1_group)
    #pge1_lbl[0].x = x_tmp  # restore x
    lcd_vpos = 1
    itms_lst = [lat, lon, gs, crs]
    led_BI_toggle()
    for msg_itm in itms_lst:
        if msg_itm == lat:
            lat_v = my_msgs.read(lat)
            dp = lat_v.find(".")
            if dp >= 0:
                if dp == 4:
                    s0 = "{: >2s}{}{:0>2s}\'{:0>2s}.{:0>2s}\"".format(lat_v[:2], degs, lat_v[2:4], lat_v[5:7],lat_v[7:])
                elif dp == 3:
                    s0 = "{: >2s}{}{:0>2s}\'{:0>2s}.{:0>2s}\" ".format(lat_v[:1], degs, lat_v[1:3], lat_v[4:6],lat_v[6:])
            s = my_msgs.read(latID) + "    " + s0
        if msg_itm == lon:
            lon_v = my_msgs.read(lon)
            dp = lon_v.find(".")
            if dp >= 0:
                if dp == 5:
                    s0 = "{: >2s}{}{:0>2s}\'{:0>2s}.{:0>2s}\"".format(lon_v[:3], degs, lon_v[3:5], lon_v[6:8], lon_v[8:])
                elif dp == 4:
                    s0 = "{: >2s}{}{:0>2s}\'{:0>2s}.{:0>2s}\" ".format(lon_v[:2], degs, lon_v[2:4], lon_v[5:7], lon_v[7:])
            s = my_msgs.read(lonID) + "   " + s0
        if msg_itm == gs:
            #gs0 = my_msgs.read(gs)
            s = "GS  {: >3d} ALT {: >5d} FT".format(round(int(float(my_msgs.read(gs)))),
                round(int(float(my_msgs.read(alt)))))
        if msg_itm == crs:
            #crs0 = my_msgs.read(crs)
            s = "CRS {:0>3d} DEGS     ".format(round(int(float(my_msgs.read(crs)))))

        pge1_lbl[lcd_vpos].text = s
        lcd_vpos += 1

    t_elapsed = (((monotonic_ns() - loop_time) + 500000)// 1000000)
    print(TAG+"Duration rx -> lcd: {} mSecs".format(t_elapsed), end="\n")

    my_msgs.clean()

    """
    #x_tmp =  pge1_lbl[3].x  # save x
    #pge1_lbl[3].x = 19
    pge1_lbl[5].text = s
    display.show(pge1_group)
    #pge1_lbl[3].x = x_tmp      # restore x
    """
    lcd_vpos += 1
    if lcd_vpos >= lcd_maxrows:
        lcd_vpos = 0

    blink_led3(led_colors_dict['off']) # Switch the external 3-color LED GREEN
    led_BI_toggle()

def main():
    global my_debug, ctrl_c_flag, pge1_group, pge1_lbl, font, color, led_pwr_ctrl
    TAG = "main(): "
    lResult = True
    cnt = 0
    print()  # Create line space between CPY status line and output of this script
    pge1_lbl = create_group()
    if my_debug:
        print(TAG+f"len(pge1_lbl)= {len(pge1_lbl)}")

    setup()
    my_board()
    if do_led3_test:
        led3_test()

    while True:
        # lcd.clear()
        sleep(2)
        s0 = ("FSUIPC7 GPS RX ", "for MSFS2020   ", "via serial     ")
        le = len(pge1_lbl)
        if le > 0:
            if my_debug:
                print(TAG+f"len(pge1_lbl)= {le}")
            le2 = len(s0)
            # set and show texts of pge1_lbl[2..4]
            for _ in range(le2):
                pge1_lbl[_+2].text = s0[_]
            display.show(pge1_group)
        else:
            raise IndexError("pge1_lbl is empty!")

        sleep(4)
        # lcd.clear()
        #clear_labels()

        if cnt == 0:
            lResult = loop()
            cnt += 1
            if lResult == False:
                if ctrl_c_flag:  # Ctrl+C has been pressed
                    break
                if my_debug == True:
                    print("loop(): setup returned with: \"{}\" in line nr: {}".format(lResult))
            break

    if pge1_group and pge1_lbl:
        # Clear display all lines
        clear_labels()
    cnt = 0
    while True:
        cnt += 1
        if cnt >= 100:
            cnt = 0
            led_BI_toggle()

if __name__ == '__main__':
    main()
