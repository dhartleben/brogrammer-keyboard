import adafruit_ssd1306
import board
import busio as io
import digitalio
import usb_hid
import os
import json
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keycode import Keycode
from adafruit_hid.keyboard_layout_us import KeyboardLayoutUS
from adafruit_hid.consumer_control import ConsumerControl
from adafruit_hid.consumer_control_code import ConsumerControlCode
from digitalio import DigitalInOut, Direction, Pull
import time
from adafruit_mcp230xx.mcp23017 import MCP23017
import rotaryio
import neopixel
import colorsys
from adafruit_debug_i2c import DebugI2C
import adafruit_bitbangio as bbio

print("Brogrammer keyboard firmware starting up")

########
# OLED #
########

print("Init i2c")
oled_i2c = io.I2C(scl=board.GP13, sda=board.GP12, frequency=400 * 1000)
print("Init oled")
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, oled_i2c)

# Run OLED timing test
start = time.monotonic()
oled.fill(0)
oled.show()
duration = time.monotonic() - start
print("Updated OLED in " + str(duration) + "sec")

oled.fill(0)
oled.text("Brogrammer", 0, 20, 1, size=2)
oled.text("Keyboard", 12, 40, 1, size=2)
oled.show()

################
# D-RGB STRIPS #
################

num_pixels = 12
strip1 = neopixel.NeoPixel(board.GP17, 12, brightness=0, auto_write=False, pixel_order=neopixel.GRB)
strip2 = neopixel.NeoPixel(board.GP16, 12, brightness=1, auto_write=False, pixel_order=neopixel.GRB)
for i in range(0, 128):
    strip1.fill((0, i, i*2))
    strip1.show()
    strip2.fill((0, i, i*2))
    strip2.show()
    time.sleep(0.005)
h = 210/360
s = 1
l = 0.5

########################
# EEPROM & IO EXPANDER #
########################

# EEPROM blocks are at 0x50-0x57 (8x 256 bytes)
io_i2c = io.I2C(scl=board.GP11, sda=board.GP10, frequency=1000000)

# https://ww1.microchip.com/downloads/en/DeviceDoc/20002213B.pdf
def eeprom_write_int(i2c, i2c_addr, block_num, mem_addr, int_value):
    debug_i2c = DebugI2C(i2c)
    while not debug_i2c.try_lock():
        pass

    control_byte = (0xA0 + (block_num << 1) + 0x0)
    bytes_to_write = [control_byte, mem_addr]
    int_value_as_bytes = list(keypresses.to_bytes(4, "big"))
    bytes_to_write.extend(int_value_as_bytes)
    try:
        debug_i2c.writeto(i2c_addr, bytearray(bytes_to_write))
        time.sleep(0.01) # Allow EEPROM time to write
    except OSError:
        debug_i2c.unlock()
        return False

    debug_i2c.unlock()
    return True

# https://ww1.microchip.com/downloads/en/DeviceDoc/20002213B.pdf
def eeprom_read_int(i2c, i2c_addr, block_num, mem_addr):
    global io_i2c
    debug_i2c = DebugI2C(i2c)
    while not debug_i2c.try_lock():
        pass

    i2c.deinit()
    i2c = bbio.I2C(scl=board.GP11, sda=board.GP10, frequency=1000000)
    debug_i2c = DebugI2C(i2c)
    while not debug_i2c.try_lock():
        pass

    try:
        # First change address
        control_byte = (0xA0 + (block_num << 1) + 0x0)
        debug_i2c.writeto(i2c_addr, bytearray([control_byte, mem_addr]))

        # Then read data at the address
        control_byte = (0xA0 + (block_num << 1) + 0x1)
        buf = bytearray(4)
        debug_i2c.writeto_then_readfrom(i2c_addr, bytearray([control_byte]), buf)
    except OSError:
        debug_i2c.unlock()
        debug_i2c.deinit()
        io_i2c = io.I2C(scl=board.GP11, sda=board.GP10, frequency=1000000)
        return False, None

    debug_i2c.unlock()
    debug_i2c.deinit()
    io_i2c = io.I2C(scl=board.GP11, sda=board.GP10, frequency=1000000)
    return True, int.from_bytes(buf, "big")

#print("Attempting to write")
#keypresses = 0
#success = eeprom_write_int(io_i2c, 0x50, 0, 0x00, keypresses)
#print("Wrote to EEPROM? ", success)
success, keypresses = eeprom_read_int(io_i2c, 0x50, 0, 0x00)
last_saved_keypresses = keypresses
print("Read from EEPROM?", success)
if success:
    print("EEPROM data read (keypresses):", keypresses)

###############
# IO EXPANDER #
###############

io_a1 = DigitalInOut(board.GP8)
io_a2 = DigitalInOut(board.GP7)
io_a3 = DigitalInOut(board.GP6)
io_a1.switch_to_output()
io_a2.switch_to_output()
io_a3.switch_to_output()
io_a1.value = 0
io_a2.value = 0
io_a3.value = 0

mcp = MCP23017(io_i2c)

#############
# GPIO PINS #
#############

# Row pins are direct GPIO pins
ROW_PINS = [
    DigitalInOut(board.GP0),  # Row 0
    DigitalInOut(board.GP1),  # Row 1
    DigitalInOut(board.GP2),  # Row 2
    DigitalInOut(board.GP3),  # Row 3
    DigitalInOut(board.GP4),  # Row 4
]
for row in ROW_PINS:
    row.switch_to_output()

# Column pins are routed through the mcp23017 IO expander
COL_PINS = [
    DigitalInOut(board.GP5),
    mcp.get_pin(8),
    mcp.get_pin(9),
    mcp.get_pin(10),
    mcp.get_pin(11),
    mcp.get_pin(12),
    mcp.get_pin(13),
    mcp.get_pin(14),
    mcp.get_pin(15),
    mcp.get_pin(0),
    mcp.get_pin(1),
    mcp.get_pin(2),
    mcp.get_pin(3),
    mcp.get_pin(4),
    mcp.get_pin(5),
    mcp.get_pin(6),
    mcp.get_pin(7),
]
for pin in COL_PINS:
    pin.switch_to_input()
    pin.pull = digitalio.Pull.UP

encoder = rotaryio.IncrementalEncoder(board.GP15, board.GP14)
last_encoder_position = None

######################
# MACROS & FUNCTIONS #
######################

class Function():
    pass

##########
# KEYMAP #
##########

Keycode.L1 = 1000
current_layer = 0
KEYMAP_L0 = [
    [
        (Keycode, Keycode.ESCAPE),
        (Keycode, Keycode.GRAVE_ACCENT),
        (Keycode, Keycode.ONE),
        (Keycode, Keycode.TWO),
        (Keycode, Keycode.THREE),
        (Keycode, Keycode.FOUR),
        (Keycode, Keycode.FIVE),
        (Keycode, Keycode.SIX),
        (Keycode, Keycode.SEVEN),
        (Keycode, Keycode.EIGHT),
        (Keycode, Keycode.NINE),
        (Keycode, Keycode.ZERO),
        (Keycode, Keycode.MINUS),
        (Keycode, Keycode.EQUALS),
        (Keycode, Keycode.BACKSPACE),
        (Keycode, Keycode.INSERT),
        (Keycode, Keycode.HOME),
    ],
    [
        (None, None),
        (Keycode, Keycode.TAB),
        (Keycode, Keycode.Q),
        (Keycode, Keycode.W),
        (Keycode, Keycode.E),
        (Keycode, Keycode.R),
        (Keycode, Keycode.T),
        (Keycode, Keycode.Y),
        (Keycode, Keycode.U),
        (Keycode, Keycode.I),
        (Keycode, Keycode.O),
        (Keycode, Keycode.P),
        (Keycode, Keycode.LEFT_BRACKET),
        (Keycode, Keycode.RIGHT_BRACKET),
        (Keycode, Keycode.BACKSLASH),
        (Keycode, Keycode.DELETE),
        (Keycode, Keycode.PAGE_UP),
    ],
    [
        (None, None),
        (Keycode, Keycode.CAPS_LOCK),
        (Keycode, Keycode.A),
        (Keycode, Keycode.S),
        (Keycode, Keycode.D),
        (Keycode, Keycode.F),
        (Keycode, Keycode.G),
        (Keycode, Keycode.H),
        (Keycode, Keycode.J),
        (Keycode, Keycode.K),
        (Keycode, Keycode.L),
        (Keycode, Keycode.SEMICOLON),
        (Keycode, Keycode.QUOTE),
        (Keycode, Keycode.ENTER),
        (None, None),  # No switch here
        (Keycode, Keycode.END),
        (Keycode, Keycode.PAGE_DOWN),
    ],
    [
        (None, None),  # No switch here
        (Keycode, Keycode.LEFT_SHIFT),
        (Keycode, Keycode.Z),
        (Keycode, Keycode.X),
        (Keycode, Keycode.C),
        (Keycode, Keycode.V),
        (Keycode, Keycode.B),
        (Keycode, Keycode.N),
        (Keycode, Keycode.M),
        (Keycode, Keycode.COMMA),
        (Keycode, Keycode.PERIOD),
        (Keycode, Keycode.FORWARD_SLASH),
        (Keycode, Keycode.RIGHT_SHIFT),
        (None, None),  # No switch here
        (None, None),  # No switch here
        (Keycode, Keycode.UP_ARROW),
        (ConsumerControlCode, ConsumerControlCode.MUTE),  # Rotary encoder push button
    ],
    [
        (None, None),  # No switch here
        (Keycode, Keycode.LEFT_CONTROL),
        (Keycode, Keycode.WINDOWS),
        (Keycode, Keycode.LEFT_ALT),
        (Keycode, Keycode.SPACE),
        (None, None),  # No switch here
        (None, None),  # No switch here
        (None, None),  # No switch here
        (Keycode, Keycode.SPACE),
        (Keycode, Keycode.L1),  # Go to Layer 1 (Fn), must match position in KEYMAP_L1
        (Keycode, Keycode.RIGHT_ALT),
        (Keycode, Keycode.APPLICATION),
        (Keycode, Keycode.RIGHT_CONTROL),
        (None, None),  # No switch here
        (Keycode, Keycode.LEFT_ARROW),
        (Keycode, Keycode.DOWN_ARROW),
        (Keycode, Keycode.RIGHT_ARROW),
    ],
]
KEYMAP_L1 = [
    [
        (None, None),
        (None, None),
        (Keycode, Keycode.F1),
        (Keycode, Keycode.F2),
        (Keycode, Keycode.F3),
        (Keycode, Keycode.F4),
        (Keycode, Keycode.F5),
        (Keycode, Keycode.F6),
        (Keycode, Keycode.F7),
        (Keycode, Keycode.F8),
        (Keycode, Keycode.F9),
        (Keycode, Keycode.F10),
        (Keycode, Keycode.F11),
        (Keycode, Keycode.F12),
        (None, None),
        (None, None),
        (None, None),
    ],
    [
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (Function, "update-tripmeter"),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
    ],
    [
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
    ],
    [
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
    ],
    [
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (Keycode, Keycode.L1),  # Must match position in KEYMAP_L0
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
        (None, None),
    ],
]
capslock = False
capslock_changed = False

################
# Keyboard/HID #
################
kbd = Keyboard(usb_hid.devices)
consumer_control = ConsumerControl(usb_hid.devices)
kbd_layout = KeyboardLayoutUS(kbd)

##################
# State tracking #
##################
pressed = []
for row in range(0, len(KEYMAP_L0)):
    pressed_row = []
    for col in range(0, len(KEYMAP_L0[row])):
        pressed_row.append(False)
    pressed.append(pressed_row)
prev_encoder_pin1 = None
prev_encoder_pin2 = None
last_keypress_time_sec = time.monotonic()
current_keypress_tripmeter = None
show_current_keypress_tripmeter = False
num_keys_down = 0

#############
# Main loop #
#############

while True:
    # Scan the key matrix and issue the appropriate keyboard commands
    # Scanning is as follows:
    #   1. For each row, set that row low and all others high
    #   2. Scan each column for key down (active low)
    try:
        for row in range(0, len(ROW_PINS)):
            row_pin = ROW_PINS[row]
            for pin in ROW_PINS:
                if pin != row_pin:
                    pin.value = True
                else:
                    pin.value = False

            for col in range(0, len(COL_PINS)):
                col_pin = COL_PINS[col]
                value = col_pin.value

                # They key is down, and was not previously detected as down
                if value == 0 and not pressed[row][col]:
                    if current_layer == 0:
                        key_type, key = KEYMAP_L0[row][col]
                    elif current_layer == 1:
                        key_type, key = KEYMAP_L1[row][col]
                    print("{} at {},{} down".format(key, row, col))

                    if key_type == Keycode:
                        if key == Keycode.L1:
                            current_layer = 1
                        else:
                            if key == Keycode.KEYPAD_NUMLOCK:
                                numlock = not numlock
                                numlock_changed = True
                            try:
                                kbd.press(key)
                            except Exception as e:
                                print(e)
                    elif key_type == str:
                        kbd_layout.write(key)
                    elif key_type == ConsumerControlCode:
                        consumer_control.send(key)
                    elif key_type == Function:
                        if key == "update-tripmeter":
                            current_keypress_tripmeter = keypresses + 1 # Account for current press
                            show_current_keypress_tripmeter = True

                    pressed[row][col] = True
                    keypresses += 1
                    num_keys_down += 1

                # The key is released, and was previously detected as down
                elif value == 1 and pressed[row][col]:
                    if current_layer == 0:
                        key_type, key = KEYMAP_L0[row][col]
                    elif current_layer == 1:
                        key_type, key = KEYMAP_L1[row][col]
                    print("{} released".format(KEYMAP_L0[row][col]))

                    if key == Keycode.L1:
                        current_layer = 0
                    else:
                        if key_type == Keycode:
                            kbd.release(key)
                    pressed[row][col] = False
                    last_keypress_time_sec = time.monotonic()
                    num_keys_down -= 1
    except OSError:
        pass

    # Handle encoder movement
    position = encoder.position
    if last_encoder_position is None or position != last_encoder_position:
        print("Encoder position: ", position)
        if last_encoder_position:
            # Layer 0 adjusts volume
            if current_layer == 0:
                if position > last_encoder_position:
                    consumer_control.send(ConsumerControlCode.VOLUME_INCREMENT)
                else:
                    consumer_control.send(ConsumerControlCode.VOLUME_DECREMENT)

            # Layer 1 adjusts brightness of LED stripss
            elif current_layer == 1:
                if position > last_encoder_position:
                    strip1.brightness += 0.05
                    strip2.brightness += 0.05
                else:
                    strip1.brightness -= 0.05
                    strip2.brightness -= 0.05

    last_encoder_position = position

    h += 0.001
    if h > 1:
        h = 0
    strip2.fill(colorsys.hls_to_rgb(h, l, s))
    strip2.show()

    if keypresses % 10 == 0 and keypresses != last_saved_keypresses:
        success = eeprom_write_int(io_i2c, 0x50, 0, 0x00, keypresses)
        print("Wrote to EEPROM? ", success)
        last_saved_keypresses = keypresses

    if (last_keypress_time_sec and time.monotonic() - last_keypress_time_sec > 3 and num_keys_down == 0) or show_current_keypress_tripmeter:
        print("Updated OLED to show keypresses")
        show_current_keypress_tripmeter = False
        last_keypress_time_sec = None

        start_oled_write = time.monotonic()
        oled.fill(0)
        oled.text("Keypresses", int((128/2) - 12*len("Keypresses")/2), 10, 1, size=2)
        if not current_keypress_tripmeter:
            text = '{:,}'.format(keypresses)
            size = 1
            if len(text)*12 < 110:
                size = 2
            oled.text(text, int((128/2) - (size*6)*len(text)/2), 40, 1, size=size)
        else:
            text = '{:,}'.format(keypresses)
            oled.text(text, int((128/2) - 6*len(text)/2), 30, 1, size=1)
            text = "Trip: " + '{:,}'.format(current_keypress_tripmeter)
            oled.text(text, int((128/2) - 6*len(text)/2), 45, 1, size=1)
            text = "Diff: " + '{:,}'.format(keypresses - current_keypress_tripmeter)
            oled.text(text, int((128/2) - 6*len(text)/2), 55, 1, size=1)
        oled.show()
        print("Updated OLED in " + str(time.monotonic() - start_oled_write) + "sec")
