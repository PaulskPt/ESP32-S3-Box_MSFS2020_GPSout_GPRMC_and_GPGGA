# SPDX-FileCopyrightText: 2023 Paulus Schulinck
#
# SPDX-License-Identifier: MIT
##############################
# Last update: 2023-07-27
# Ultimate addition: 'capture' of 'red circle' button to exit app.
#
import board
import busio
import digitalio
import sys, os, gc
import time  # from time import time, sleep, monotonic, monotonic_ns
#import adafruit_dotstar as dotstar
#from displayio import Group
import displayio
from adafruit_display_text import label
from adafruit_bitmap_font import bitmap_font

# General debug flag
my_debug = False
# Other global flags
ctrl_c_flag = False
use_led3 = True
do_led3_test = False
use_touch = True
use_wifi = True
use_ping = True

i2c = None
ssid = None
password = None
pool = None
requests = None
tt = None
ip = None
s_ip = None
location = None
tz_offset = None
rtc = None

if use_wifi:
    import wifi
    import ssl
    import ipaddress
    import socketpool
    from secrets import secrets
    import adafruit_requests
    from rtc import RTC
    ssid = secrets["ssid"]
    password = secrets["password"]

if use_touch:
    import adafruit_tt21100


my_os = None  # will hold a list containing O.S. info
my_machine = None
#               like fw version, machine name.
#               Data collected through function: get_os_info()

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
if my_debug:
    print(f"\ninit(): board.board_id= {id}")
if id == 'espressif_esp32s3_box':
    i2c = busio.I2C(board.SCL, board.SDA)
    TX = board.G43  # U0TXD  # board.G43
    RX = board.G44  # U0RXD  # board.G44
else:
    use_touch = False
    TX = None
    RX = None

if use_touch:
    # Create library object (named "tt") using a Bus I2C port
    tt = adafruit_tt21100.TT21100(i2c)
    if my_debug:
        print(f"init(): dir(tt)= {dir(tt)}")
else:
    TX = None
    RX = None


while not i2c.try_lock():
    pass

# the code in the next lines reported the following addresses:
# I2C addresses found: ['0x18', '0x24', '0x40', '0x68']

try:
    print(
        "\nI2C addresses found:",  #
        [hex(device_address) for device_address in i2c.scan()],
    )
    time.sleep(2)

finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
    i2c.unlock()

time.sleep(10)

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

weekdays = {0:"Monday", 1:"Tuesday",2:"Wednesday",3:"Thursday",4:"Friday",5:"Saturday",6:"Sunday"}

if use_touch:
    touch_int   = digitalio.DigitalInOut(board.CTP_INT) # Control Touch Point Interrupt line
    touch_int.pull = digitalio.Pull.UP
    red_circle_flag = False
    touch_iter = 0
    """
    touch_x = 0
    touch_y = 0
    touch_id = 0  # this is used in case of multi-touch
    touch_pr = 0
    touch_iter = 0
    """
else:
    touch_int = None
    red_circle_flag = None
    touch_iter = None
    """
    touch_x = None
    touch_y = None
    touch_id = None
    touch_pr = None
    touch_iter = None
    """

if use_led3:
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
    LED_R = None
    LED_G = None
    LED_B = None
    led3_red = None
    led3_green = None
    led3_blue = None
    HIGH = None
    LOW = None

biLdIsOn = False # Flag for the FeatherS2 built-in blue led

if my_debug:
    print()
    print(f"globals: TX= {TX}, RX= {RX}")
uart = busio.UART(TX, RX, baudrate=4800, timeout=0, receiver_buffer_size=151)  # board.RX, board.TX)
#uart = board.UART()

rtc = RTC()

# release any currently configured displays
#displayio.release_displays()
start_t= time.monotonic()
start_0 = start_t # remember (and don't change this value!!!)
time_received = False
display = board.DISPLAY
pge1_group = None  # group to hold our texts
pge1_lbl = []      # list of labels
max_text_len = 25
#fn = "/fonts/helvR12.pcf"  # This font has a degrees sign
fn = "/fonts/helvR12-ISO8859-1.pcf" # This font has a degrees sign
font = bitmap_font.load_font(fn)
color = 0x00FF00
scale = 2

display.rotation = 0 # set the rotation (upside-down)
rotation = display.rotation  # get the current rotation

dx = 0  #100
dy = 10

# Note: with this display rotation values are 0, 90, 180, 270 !
if rotation == 0 or rotation == 180:
    WIDTH = display.width   # get the display width (240)
    HEIGHT = display.height  # get the display height (320)
    disp_pos_dict = { 0: (dx+290, dy),
        1: (dx, dy+40),
        2: (dx, dy+80),
        3: (dx, dy+120),
        4: (dx, dy+160),
        5: (dx, dy+200)
    }
elif rotation == 90 or rotation == 270:
    WIDTH = display.height  # get the display height (320)
    HEIGHT = display.width  # get the display width (240)
    disp_pos_dict = { 0: (dx, dy+290),
    1: (dx+40, dy),
    2: (dx+80, dy),
    3: (dx+120, dy),
    4: (dx+160, dy),
    5: (dx+200, dy)
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
lac_Taxying = False
lacTaxyMsgShown = False
acStopInitMonot = 0
acStopInterval = 6000 # mSec

# Buffers
rx_buffer_len = 160
rx_buffer = bytearray(rx_buffer_len * b'\x00')

# Classes
my_msgs = gps_msgs()

def ck_touch():
    # if the screen is being touched print the touches
    global touch_iter, red_circle_flag
    TAG = "ck_touch(): "
    touch_iter += 1
    if touch_iter >= 1000:
        touch_iter = 0
        if my_debug:
            print(TAG+"resetting touch iteration counter to 0")
    # if the screen is being touched print the touches
    ts = None
    n = tt.touched
    if n > 0:
        if my_debug:
            print(TAG+f"nr of touches= {n}")
        ts = tt.touches
        if my_debug:
            print(f"type(ts)= {type(ts)}")
            print(TAG+f"screen touched at: ({ts[0]['x']},{ts[0]['y']}). Pressure: {ts[0]['pressure']}")
    else:
        if my_debug:
            print(TAG+f"touch iteration: {touch_iter}. Nr touch(es)= {n}")

        if touch_int.value == False:
            # No normal button has been pressed but the button (CTP_INT) interrupt line is False,
            # so, a virtual button, probably the red circle button, has been pressed.
            # Maybe we can reset the CTP_INT line by clearing the touch buffer.
            red_circle_flag = True
            print(TAG+"red circle button has been pressed.")
            if my_debug:
                print(TAG+f"red_circle_flag set. Value= {red_circle_flag}")
            clr_touch()
            if my_debug:
                print(TAG+f"touch buffer cleared. Status of CTP_INT= {touch_int.value}")
    return ts

def clr_touch():
    # global touch_x, touch_y, touch_id, touch_pr, tt, touch_int
    TAG="clr_touch(): "
    """
    touch_x = 0
    touch_y = 0
    touch_id = 0
    touch_pr = 0
    """
    # Clear the touch buffer
    cnt = 0
    ts = []
    while True:
        ts = tt.touches
        if my_debug:
            print(TAG+f"ts= {ts}, type(ts)= {type(ts)}")
        if isinstance(ts, list):
            if len(ts) == 0:
                break
        if cnt == 0:
            time.sleep(0.2)
        cnt += 1
        if cnt >= 10:
            if not my_debug:
                print(TAG+f"touch buffer read {cnt} times.")
            break


def wifi_is_connected():
    global ip, s_ip
    ret = False

    ip = wifi.radio.ipv4_address

    if ip:
        s_ip = str(ip)
        le_s_ip = len(s_ip)

    if s_ip is not None and len(s_ip) > 0 and s_ip != '0.0.0.0':
        ret = True
    return ret

def do_connect():
    global ip, s_ip, pool, ssid, password

    print(f"\nTrying to connect to \'{ssid}\'")

    print(f"Connecting to \'{ssid}\'")
    wifi.radio.connect(ssid=ssid, password=password)
    pool = socketpool.SocketPool(wifi.radio)
    connected = False
    s2 = ''

    wifi.radio.connect(ssid=ssid, password=password)

    if wifi_is_connected():
        connected = True
        s2 = ''
    else:
        s2 = "Not "

    print(f"\n{s2}connected to: \'{ssid}\'")

    if my_debug:
        print("s_ip= \'{}\'".format(s_ip))

    if use_ping and connected:
        if not pool:
            pool = socketpool.SocketPool(wifi.radio)
        addr_idx = 1
        addr_dict = {0:'LAN gateway', 1:'google.com'}
        info = pool.getaddrinfo(addr_dict[addr_idx], 80)
        addr = info[0][4][0]
        print(f"Resolved google address: \'{addr}\'")
        ipv4 = ipaddress.ip_address(addr)
        for _ in range(10):
            result = wifi.radio.ping(ipv4)
            if result:
                print("Ping google.com [%s]: %.0f ms" % (addr, result*1000))
                break
            else:
                print("Ping no response")

def open_socket():
    global pool, requests
    pool = socketpool.SocketPool(wifi.radio)
    requests = adafruit_requests.Session(pool, ssl.create_default_context())

def free_socket():
    global pool, requests
    requests._free_sockets()

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

def my_board():
    global my_os, id
    if my_debug:
        if my_os:
            print("OS: {}.".format(my_os), end='\n')
        if id:
            print("CPU ID: {}.".format(id), end='\n')

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
            color= 0xFFFF00,
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

def clear_labels():
    global pge1_group, pge1_lbl, max_text_len
    s = ' '*max_text_len
    for _ in range(len(pge1_lbl)):
        pge1_lbl[_].text = s
    display.show(pge1_group)

def setup():
    global lcd, uart, ssid, degreesChar, ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY, location, tz_offset
    TAG = "setup(): "
    if my_debug:
        print(TAG+"...")
    # Here, is necessary, initialization of the esp32-s3-box display
    #create_group()

# Get our username, key and desired timezone
    ADAFRUIT_IO_USERNAME = secrets["aio_username"]
    ADAFRUIT_IO_KEY = secrets["aio_key"]
    location = secrets.get("timezone", None)


    #open_socket()

    # This part copied from I:/PaulskPt/Adafruit_DisplayIO_FlipClock/Examples/displayio_flipclock_ntp_test2_PaulskPt.py
    lt = secrets.get("LOCAL_TIME_FLAG", None)
    if lt is None:
        use_local_time = False
    else:
        lt2 = int(lt)
        if my_debug:
            print("lt2=", lt2)
        use_local_time = True if lt2 == 1 else False

    if use_local_time:
        location = secrets.get("timezone", None)
        if location is None:
            location = 'Not set'
            tz_offset = 0
        else:
            tz_offset0 = secrets.get("tz_offset", None)
            if tz_offset0 is None:
                tz_offset = 0
            else:
                tz_offset = int(tz_offset0)
    else:
        location = 'Etc/GMT'
        tz_offset = 0
    # end of part copied from flipclock ---------------------------------------------


    if led_state == LOW: # Switch off the RGB LED
        led_toggle()
        #neopix_led_off()

    #lcd_chr_test()  # print all the characters in the lcd rom

    if uart:
        uart.reset_input_buffer()  # Clear the rx buffer

    time.sleep(1)  # <--------------- DELAY ---------------

    get_os_info()  # Collect O.S. info. Put in global variable my_os and (partly) in my_machine

    # because we have the file .env on disk
    # a WiFi connection perhaps already exists
    if not wifi_is_connected():
        do_connect()
    else:
        print(TAG+f"WiFi already connected to: \'{ssid}\'")

def loop():
    global startup, start_t, lp_cnt, led_state, biLdIsOn, msg_nr, ctrl_c_flag, diagn_dict, pge1_group, pge1_lbl, scale, font, color, msg_rx_ok, msg_rx_fail, lacTaxyMsgShown, use_touch
    TAG = "loop(): "
    if my_debug:
        print(TAG+"...")
    lRetval = True  # assume positive
    chrs_rcvd = 0
    lstop = False
    curr_t = None
    elapsed_t = None
    interval_t = 600  # 10 minutes

    if my_machine:
        print(TAG+"my_machine= \"{}\"".format(my_machine))
        n1 = my_machine.find("ESP32S")
        n2 = my_machine.find("with")
        if n1 > 0 and n2 >=0:
            s3 = my_machine[:n2]
            s4 = my_machine[n2:] # n2+1]+" "+my_machine[n2+5:]
        else:
            s3 = my_machine[:19]  # Not more than 20 characters
            s4 = ''

        #lcd.write(s3)
        #print(TAG+"my_machine (cut)= \"{}\"".format(s))
    else:
        s3 = sys.platform
        s4 = ''

    s0 = ("MSFS 2020      ", "GPRMC/GPGGA data RX", "Platform ", s3, s4)
    # pge1_lbl[1..5]
    le = len(s0)
    for _ in range(le):
        pge1_lbl[_+1].text = s0[_]
    display.show(pge1_group)

    time.sleep(5)

    # Clear only texts of pge1_lbl[3..5]
    s = ' '*max_text_len
    for _ in range(3):
        pge1_lbl[_+3].text = s
    display.show(pge1_group)

    s0 = ("Touch the      ", "red circle button ", "to exit app    ", "", "")
    # pge1_lbl[1..5]
    le = len(s0)
    for _ in range(le):
        pge1_lbl[_+1].text = s0[_]
    display.show(pge1_group)

    time.sleep(5)

    # Clear only texts of pge1_lbl[3..5]
    s = ' '*max_text_len
    for _ in range(3):
        pge1_lbl[_+3].text = s
    display.show(pge1_group)

    print()
    print("MSFS2020 GPS GPRMC data reception decoder script")
    print("(https://github.com/PaulskPt/ESP32-S3-Box_MSFS2020_GPSout_GPRMC_and_GPGGA)")
    print("\nNumber of loops in this run: {}".format(max_lp_cnt))
    chrs_rcvd = 0
    print("........................", end="\n")
    clr_touch() # This also clears the CTP_INT line
    while True:
        try:
            curr_t = time.monotonic()
            elapsed_t = int(float(curr_t - start_t))

            if use_wifi:
                w_cnt = 0
                while not wifi_is_connected():
                    do_connect()
                    w_cnt += 1
                    if w_cnt >= 10:
                        print(TAG+"WiFi connection was lost. Failed to re-establish")
                        break
            print(TAG+f"elapsed time since script (re-)start: {curr_t-start_0}")
            print(TAG+f"elapsed_t % {interval_t} = {elapsed_t % interval_t}")
            if wifi_is_connected():
                if elapsed_t >= 20 and elapsed_t % interval_t <= 5:  # leave a margin
                    start_t = curr_t
                    gc.collect()
                    get_time_fm_AIO()
                    get_dt_fm_rtc()
            gc.collect()
            if my_debug:
                print(TAG+f"use_touch= {use_touch}")
            ck_touch()  # Check if we have to exit the app by touch the red circle button
            print(TAG+f"red_circle_flag= {red_circle_flag}")
            if red_circle_flag:
                raise KeyboardInterrupt
            lp_cnt += 1
            print("\nStart of loop {}".format(lp_cnt), end="\n")

            if led_state == LOW: # Switch off the dotstar RGB LED
                led_toggle()
            if startup == -1:
                uart.reset_input_buffer()
                #pge1_lbl[5].text = "About to receive..."
                s0 = ("", "", "", "", "About to receive...")
                # pge1_lbl[1..5]
                le = len(s0)
                for _ in range(le):
                    pge1_lbl[_+1].text = s0[_]
                display.show(pge1_group)
            wait_cnt = 0
            chrs_rcvd = ck_uart()
            print(TAG+"characters rcvd: ", chrs_rcvd)
            if chrs_rcvd == -1:
                raise KeyboardInterrupt
            if chrs_rcvd > 0:
                lResult = split_types()
                print(TAG+"split_types() result = {}".format(lResult))
                if lResult == True:
                    msg_rx_ok += 1
                    if not is_ac_stopped():
                        if is_ac_taxying():
                            lacTaxyMsgShown = False
                            startup = 0
                        else:
                            if startup == -1:
                                clear_labels()
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
                            time.sleep(0.20) # Wait a bit so we prob don't have 2 incomplete msgs
                            print(TAG+f"gc.mem_free()= {gc.mem_free()}")
                            chrs_rcvd = 0
                            #time.sleep(0.1)
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
                        startup = 0
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
            if not red_circle_flag:
                s0 = ("\'Ctrl+C\' pressed.", "Going to quit...")
            else:
                s0 = ("\'red circle button\' pressed.", "Going to quit...")
            print(s0[0], end='')
            print(s0[1], end='\n')
            pge1_lbl[4].text = s0[0]
            pge1_lbl[5].text = s0[1]
            display.show(pge1_group)
            time.sleep(5)
            lRetval = False
            break
    #time.sleep(0.5)
    return lRetval

def ck_uart():
    global rx_buffer, msg_nr, loop_time, diagn_dict, nRMC, nGGA
    TAG = 'ck_uart(): '
    nr_bytes = i = 0
    delay_ms = 0.3
    if use_diagnosics:
        rx_wait_start = time.monotonic_ns()
    while True:
        ck_touch()
        if red_circle_flag:
            nr_bytes = -1
            break
        try:
            rx_buffer = bytearray(rx_buffer_len * b'\x00')
            nr_bytes = uart.readinto(rx_buffer)
            #for i in range(5):
            #    time.sleep(delay_ms)
            loop_time = time.monotonic_ns()
            if nr_bytes is None:
                if my_debug:
                    print(TAG+"nr_bytes is None")
                time.sleep(delay_ms)
                continue
            if not nr_bytes:
                if my_debug:
                    print(TAG+"nr_bytes=", nr_bytes)
                time.sleep(delay_ms)
                continue
            if nr_bytes > 1:
                if not my_debug:
                    print(TAG+"nr of bytes= ", nr_bytes)
                if nr_bytes < 146: # Try to cut short checking. A full/OK msg has 153 characters
                    time.sleep(delay_ms)
                    continue
                print(TAG+"rcvd data: {}".format(rx_buffer),end="\n")
                # Check for '*' in tail. If present, the message is very probably complete.
                tail_part = -20
                tp = rx_buffer[tail_part:]
                print(TAG+f"rx_buffer[{tail_part}:]= \'{tp}\'")
                n1 = tp.find('*') # find '*' in tail 10 characters. If not found, loop
                #n1 = rx_buffer.find('*')
                print(TAG+f"n1 = {n1}")
                if n1 < 0: # no '*' in tail
                    time.sleep(delay_ms)
                    continue
                else:
                    n2 = rx_buffer.find('$GPRMC')
                    # print(TAG+f"n2 = {n2}")
                    if n2 < 0:
                        time.sleep(delay_ms)
                        continue
                    else:
                        nRMC = n2
                    n3 = rx_buffer.find('$GPGGA')
                    # print(TAG+f"n3 = {n3}")
                    if n3 < 0:
                        time.sleep(delay_ms)
                        continue
                    else:
                        nGGA = n3
                    n4 = int(nr_bytes*0.75) # get 3/4 of length of characters received
                    # print(TAG+f"n4 = {n4}")
                    if not my_debug:
                        #print(TAG+"n1 = {}, n2 = {}, n3= {}, n4 ={}".format(n1, n2, n3, n4))
                        print(TAG+"* at {}, $GPRMC at {}, $GPGGA at {}, n4 ={}".format(n1+tail_part+1, n2, n3, n4))
                    if n2 > n4: # check if $GPRMC occurs at more than 3/4 of the characters received
                        time.sleep(delay_ms)
                        continue
                    else:
                        # print(TAG+"returning with {} nr of bytes".format(nr_bytes))
                        if use_diagnosics:
                            rx_wait_stop = time.monotonic_ns()
                            rx_wait_duration = float((rx_wait_stop - rx_wait_start) / 1000000000) # convert nSec to mSec
                            diagn_dict[msg_nr+1] = {0: rx_wait_duration, 1: -1} # add a key/value pair for diagnostics
                            print(TAG+"it took {:6.2f} seconds for a complete msg ($GPRMC & $GPGGA) to be received".format(rx_wait_duration))
                        #return nr_bytes
                        break
            elif nr_bytes == 1:
                if rx_buffer[0] == b'\x00':
                    time.sleep(delay_ms)
                    i += 1
                    if i % 1000 == 0:
                        print("Waiting for uart line to become ready")
            else:
                empty_buffer()
                time.sleep(delay_ms)
                continue
        except KeyboardInterrupt:
            nr_bytes = -1
    return nr_bytes

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

def led_BI_toggle():
    pass

    """
    global led, biLdIsOn
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
        # Sleep for 15ms so the colour cycle isn't too fast
        time.sleep(0.015)
        # [0] = color_c_arr[led_colors_dict['green']]  # Set the color of the builtin LED to GREEN
        # .show()
        blink_led3(led_colors_dict['green']) # Switch the external 3-color LED GREEN

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
            time.sleep(2)

    clear_labels()
    blink_led3("off")  # switch off the external RGB led
    print(TAG+"end of test")

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

def ck_gs():
    TAG= "ck_gs(): "
    t_gs = my_msgs.read(_gs)
    le = len(t_gs)
    if not my_debug:
        print(TAG,"value of gs = {}, len(gs) = {}".format(t_gs, le), end='\n')
    if le == 0: # check for t_gs == "". This happened sometimes!
        v_gs = 0
    else:
        v_gs = round(int(float(t_gs)))
    return v_gs

def is_ac_stopped():
    global lac_Stopped, lacStopMsgShown, lacTaxyMsgShown, acStopInitMonot, acStopInterval, pge1_group, pge1_lbl
    TAG = "is_ac_stopped(): "
    s = "Aircraft is stopped or parked"
    #gs = 5
    t_elapsed = 0
    lelapsed = False
    v_gs = ck_gs()
    if not my_debug:
        print(TAG,"value of v_gs = {}".format(v_gs), end='\n')
    if v_gs == 0:
        if not lac_Stopped:
            acStopInitMonot = 0
        lac_Stopped = True
    else:
        lac_Stopped = False
        lacStopMsgShown = False
        lacTaxyMsgShown = False
        acStopInitMonot = 0
        return lac_Stopped

    if lac_Stopped:
        currMonot = time.monotonic_ns()
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
                # It takes about ten seconds before this message is shown after aircraft is stopped
                pge1_lbl[1].text = ' '*max_text_len
                pge1_lbl[2].text = 'Airplane is stopped'
                pge1_lbl[3].text = 'or parked'
                pge1_lbl[4].text = ' '*max_text_len
                pge1_lbl[5].text = ' '*max_text_len
                display.show(pge1_group)
                lacStopMsgShown = True
            print(TAG+s, end = '\n') # Alway print to REPL (it does almost immediately)
    return lac_Stopped

def is_ac_taxying():
    global lac_Stopped, lac_Taxying, lacTaxyMsgShown, v_gs, pge1_group, pge1_lbl
    TAG= "is_ac_taxying(): "
    s = "Airplane is taxying"
    v_gs = ck_gs()
    if v_gs > 0.2 and v_gs <= 30.0:  # keep margin. Sometimes while parked the gs can be 0.1
        lac_Stopped = False
        lac_Taxying = True
        if lacTaxyMsgShown == False:
            pge1_lbl[1].text = ' '*max_text_len
            pge1_lbl[2].text = s
            pge1_lbl[4].text = ' '*max_text_len
            pge1_lbl[5].text = ' '*max_text_len
            display.show(pge1_group)
            lacTaxyMsgShown = True
        pge1_lbl[3].text = 'Speed = {} kts'.format(v_gs) # *max_text_len
        print(TAG+s)
    else:
        lac_Taxying = False
        # t_parked_init = 0.0
        #led_toggle()  # we toggle elsewhere
    return lac_Taxying

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
        degs = chr(0xb0)  # if font = "/fonts/helvR12.pcf"
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

    t_elapsed = (((time.monotonic_ns() - loop_time) + 500000)// 1000000)
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

def get_time_fm_AIO():
    global time_received, ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY
    TAG = "get_time_fm_AIO(): "
    TIME_URL = "https://io.adafruit.com/api/v2/{:s}/integrations/".format(ADAFRUIT_IO_USERNAME)
    TIME_URL += "time/strftime?x-aio-key={:s}&tz={:s}".format(ADAFRUIT_IO_KEY, location)
    TIME_URL += "&fmt=%25Y-%25m-%25d+%25H%3A%25M%3A%25S.%25L+%25j+%25u+%25z+%25Z"
    dst = ''
    print(TAG+"ip=", ip)
    if not wifi_is_connected():
        do_connect()
    if wifi_is_connected():
        try:
            open_socket()
            time.sleep(0.5)
            response = requests.get(TIME_URL)
            if response:
                n = response.text.find("error")
                if n >= 0:
                    print("AIO returned an error: ", response)
                else:
                    print("-" * 47)
                    print("Time= ", response.text)
                    print("-" * 47)
                    time_received = True
                    s = response.text
                    s_lst = s.split(" ")
                    #print(f"s_lst= {s_lst}")
                    n = len(s_lst)
                    if n > 0:
                        dt1 = s_lst[0]
                        tm = s_lst[1]
                        yday = s_lst[2]
                        wday = s_lst[3]
                        tz = s_lst[4]
                        dst = -1  # we don't use isdst
                        yy =int(dt1[:4])
                        mo = int(dt1[5:7])
                        dd = int(dt1[8:10])
                        #print(f"tm= {tm}")
                        hh = int(tm[:2])
                        mm = int(tm[3:5]) # +mm_corr # add the correction
                        ss = int(round(float(tm[6:8])))
                        print(f"ss= {ss}")
                        yd = int(yday) # day of the year
                        wd = int(wday)-1 # day of the week -- strftime %u (weekday base Monday = 1), so correct because CPY datetime uses base 0
                        #sDt = "Day of the year: "+str(yd)+", "+weekdays[wd]+" "+s_lst[0]+", "+s_lst[1][:5]+" "+s_lst[4]+" "+s_lst[5]
                        sDt = "Day of the year: {}, {} {} {} {} {}".format(yd, weekdays[wd], s_lst[0], s_lst[1][:5], s_lst[4], s_lst[5])
                        if not my_debug:
                            print(TAG+"sDt=", sDt)
                        """
                            NOTE: response is already closed in func get_time_fm_aio()
                            if response:
                                response.close()  # Free resources (like socket) - to be used elsewhere
                        """
                        # Set the internal RTC
                        tm2 = (yy, mo, dd, hh, mm, ss, wd, yd, dst)
                        if not my_debug:
                            print(TAG+f"tm2= {tm2}")
                        tm3 = time.struct_time(tm2)
                        if my_debug:
                            print(TAG+"dt1=",dt1)
                            print(TAG+"yy ={}, mo={}, dd={}".format(yy, mm, dd))
                            print(TAG+"tm2=",tm2)
                            print(TAG+"tm3=",tm3)
                        rtc.datetime = tm3 # set the built-in RTC
                        print(TAG+"built-in rtc synchronized with Adafruit Time Service date and time")
                        if my_debug:
                            print(TAG+" Date and time splitted into:")
                            for i in range(len(s_lst)):
                                print("{}: {}".format(i, s_lst[i]))
                response.close()
                free_socket()
        except OSError as exc:
            print(TAG+"OSError occurred: {}, errno: {}".format(exc, exc.args[0]), end='\n')

def get_dt_fm_rtc():
    TAG = "get_dt_fm_rtc(): "
    """
        Get the datetime from the built-in RTC
        After being updated (synchronized) from the AIO time server;
        Note: the built-in RTC datetime gives always -1 for tm_isdst
              We determine is_dst from resp_lst[5] extracted from the AIO time server response text
    """
    ct = rtc.datetime  # read datetime from built_in RTC
    # print(TAG+f"datetime from built-in rtc= {ct}")
    # weekday (ct[6]) Correct because built-in RTC weekday index is different from the AIO weekday
    #                              yy     mo     dd
    dt = "{}-{:02d}-{:02d}".format(ct[0], ct[1], ct[2])
    #                           hh     mm
    tm = "{:02d}:{:02d}".format(ct[4], ct[5])
    tz = location
    tz_off = tz_offset
    #                                                                                                              yd                   yy     mo     dd     hh     mm            is_dst
    sDt = "YearDay: {}, WeekDay: {} {:4d}-{:02d}-{:02d}, {:02d}:{:02d}, timezone offset: {} Hr, is_dst: {}".format(ct[7], weekdays[ct[6]], ct[0], ct[1], ct[2], ct[3], ct[4], tz_off, ct[8])
    if not my_debug:
        print(f"get_dt_fm_rtc(): date time from built-in rtc: {sDt}")

def main():
    global my_debug, ctrl_c_flag, pge1_group, pge1_lbl, font, color, led_pwr_ctrl, red_circle_flag
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
        try:
            time.sleep(2)
            s0 = ("FSUIPC7 GPS RX ", "for MSFS2020   ", "via serial     ")
            le = len(pge1_lbl)
            if le > 0:
                if my_debug:
                    print(TAG+f"len(pge1_lbl)= {le}")
                le2 = len(s0)
                # set and show texts of pge1_lbl[1..3]
                for _ in range(le2):
                    pge1_lbl[_+1].text = s0[_]
                display.show(pge1_group)
            else:
                raise IndexError("pge1_lbl is empty!")

            time.sleep(4)

            #clear_labels()

            if cnt == 0:
                gc.collect()
                lResult = loop()
                cnt += 1
                if lResult == False and ctrl_c_flag == True:
                    # Ctrl+C has been pressed
                    if not my_debug:
                        print(TAG+f"loop() returned with: \"{lResult}\"")
                        raise KeyboardInterrupt
        except KeyboardInterrupt:
            if not red_circle_flag and ctrl_c_flag:
                print(TAG+"Keyboard interrupt. Exiting...")
            elif red_circle_flag:
                print(TAG+"red circle button pressed. Exiting...")
            break

    if pge1_group and pge1_lbl:
        # Clear display all lines
        clear_labels()
    sys.exit()


if __name__ == '__main__':
    main()
