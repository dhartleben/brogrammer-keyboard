##################################
#                                #
#  BROGRAMMER KEYBOARD FIRMWARE  #
#  Written by Devin Hartleben    #
#  For Brogrammer Keyboard v1.0  #
#                                #
##################################

# Built-in imports
from machine import Pin, I2C, SoftI2C
import array
import utime
import _thread
import gc
import rp2

# Third-party imports
import ssd1306
from keyboard import Keyboard, Keycode
from consumer_control import ConsumerControl
import mcp23017
from rotary_irq_rp2 import RotaryIRQ
import colorsys

###########
# Classes #
###########

# This represents a single address in EEPROM (comprised of both block (3 bits) and addr (8 bits)
# See https://ww1.microchip.com/downloads/en/DeviceDoc/20002213B.pdf
class MemAddr:
    def __init__(self, *, block, addr):
        self.block = block
        self.addr = addr

# This is used within the KEYMAP_*s to indicate that the key mapping
# corresponds to a function 
class Function:
    pass

# This simplifies/cleans up sharing variables between the two cores
class SharedVariable:
    def __init__(self, *, value):
        self.lock = _thread.allocate_lock()
        self.value = value
    
    def get_value(self):
        with self.lock:
            return self.value

    def set_value(self, value):
        with self.lock:
            self.value = value

#############
# Constants #
#############

OWNER = "Jocy"
NUM_LEDS = 30
MEM_ADDR_TOTAL_KEYRESSES = MemAddr(block=0, addr=0x00)
MEM_ADDR_KEYPRESS_TRIPMETER = MemAddr(block=1, addr=0x00)

####################
# Shared Variables #
####################
# These variables are intended to be shared by both cores

sv_total_keypresses = SharedVariable(value=None)
sv_keypress_tripmeter = SharedVariable(value=None)
sv_show_tripmeter = SharedVariable(value=False)
sv_last_keypress = SharedVariable(value=None)
sv_force_eeprom_write = SharedVariable(value=False)
sv_show_debug_info = SharedVariable(value=False)
sv_matrix_scans_per_sec = SharedVariable(value=None)
sv_hue = SharedVariable(value=210/360)
sv_saturation = SharedVariable(value=1)
sv_lightness = SharedVariable(value=0.5)

##############################
# Pin mappings               #
#   and                      #
# Hardware-related resources #
##############################

# # I2C pins
oled_scl = Pin(13, Pin.OUT)
oled_sda = Pin(12)
io_scl = Pin(11, Pin.OUT)
io_sda = Pin(10)
io_a1, io_a2, io_a3 = Pin(8, Pin.OUT), Pin(7, Pin.OUT), Pin(6, Pin.OUT)
io_a1.value(False)
io_a2.value(False)
io_a3.value(False)

# This i2c is used by:
#   - OLED screen
oled_i2c = I2C(id=0, scl=oled_scl, sda=oled_sda, freq=100 * 1000)

# The display is a SSD1306-based OLED from DFRobot, which is 128x64
oled = ssd1306.SSD1306_I2C(128, 64, oled_i2c)
oled.fill(0)
oled.show()

# This i2c is used by:
#   - IO Expander (MCP23017)
#   - EEPROM (24LC16)
#
# The reason we use a SoftI2C instead of a hardware-supported I2C is because
# we implement custom I2C EEPROM communication below.
io_i2c = SoftI2C(io_scl, io_sda, freq=100 * 1000)
io_i2c_lock = _thread.allocate_lock()

# The IO expander is used since the Pico doesn't have quite enough GPIO for
# the 5x17 switch matrix
mcp = mcp23017.MCP23017(io_i2c, 0x20)

# The rotary encoder is attached to two GPIO pins directly.
# "This is a robust implementation providing effective debouncing of
# encoder contacts. It uses two GPIO pins configured to trigger
# interrupts, following Ben Buxton's implementation"
# https://github.com/miketeachman/micropython-rotary
# rotary = RotaryIRQ(pin_num_clk=14,
#               pin_num_dt=15,
#               min_val=None,
#               max_val=None,
#               reverse=False,
#               pull_up=True,
#               range_mode=RotaryIRQ.RANGE_UNBOUNDED)
# rotary.reset()
# last_rotary_value = 0
rotary_a = Pin(14, Pin.IN, Pin.PULL_UP)
rotary_b = Pin(15, Pin.IN, Pin.PULL_UP)
last_rotary_a = rotary_a.value()
last_rotary_b = rotary_b.value()
running_rotary_values_a = 0
running_rotary_values_b = 0

# Row pins are direct GPIO pins
ROW_PINS = [
    Pin(0, Pin.OUT),
    Pin(1, Pin.OUT),
    Pin(2, Pin.OUT),
    Pin(3, Pin.OUT),
    Pin(4, Pin.OUT),
]

# Column pins are routed through the mcp23017 IO expander
# The index into COL_PINS (ie col number in switch matrix)
# contains the value which is the bit position in a two-byte
# number which corresponds to a read of all values of the IO expander
# The LSB corresponds to IO0 and the MSB corresponds to IO15.
COL_PINS = [
    Pin(5, Pin.IN, Pin.PULL_UP),
    mcp[8],
    mcp[9],
    mcp[10],
    mcp[11],
    mcp[12],
    mcp[13],
    mcp[14],
    mcp[15],
    mcp[0],
    mcp[1],
    mcp[2],
    mcp[3],
    mcp[4],
    mcp[5],
    mcp[6],
    mcp[7],
]
for col in range(1, len(COL_PINS)):
    COL_PINS[col].input(pull=1)

#############
# LED Strip #
#############

brightness = 0.75

# See http://www.pibits.net/code/raspberry-pi-pico-and-neopixel-example-in-micropython.php
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()
 
# Create the StateMachine with the ws2812 program.
sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(16))
 
# Start the StateMachine, it will wait for data on its FIFO.
sm.active(1)
 
# Display a pattern on the LEDs via an array of LED RGB values.
ar = array.array("I", [0 for _ in range(NUM_LEDS)])
 
def show_pixels():
    dimmer_ar = array.array("I", [0 for _ in range(NUM_LEDS)])
    for i,c in enumerate(ar):
        r = int(((c >> 8) & 0xFF) * brightness)
        g = int(((c >> 16) & 0xFF) * brightness)
        b = int((c & 0xFF) * brightness)
        dimmer_ar[i] = (g<<16) + (r<<8) + b
    sm.put(dimmer_ar, 8)
    utime.sleep_ms(10)
 
def set_pixel(i, color):
    # Color should be a triple (r, g, b)
    ar[i] = (color[1]<<16) + (color[0]<<8) + color[2]
 
def set_all_pixels(color):
    # Color should be a triple (r, g, b)
    for i in range(len(ar)):
        set_pixel(i, color)

######################
# Keyboard resources #
######################

kbd = Keyboard()
kbd.release_all()

consumer_control = ConsumerControl()

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
        (Function, "reset-tripmeter"),
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
        (Function, "show-tripmeter"),
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
        (ConsumerControl, ConsumerControl.MUTE),  # Rotary encoder push button
        #(ConsumerControlCode, ConsumerControlCode.MUTE),  # Rotary encoder push button
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
        (Function, "reset-total-keypresses"),
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
        (Function, "show-debug"),
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


####################
# EEPROM resources #
####################

def eeprom_read_int(mem_addr: MemAddr):
    # https://ww1.microchip.com/downloads/en/DeviceDoc/20002213B.pdf
    io_i2c.start()
    control_byte = (0xA0 + (mem_addr.block << 1) + 0x0)
    num_acks = io_i2c.write(bytearray([control_byte, mem_addr.addr]))
    if num_acks != 2:
        return False, None
    
    control_byte = (0xA0 + (mem_addr.block << 1) + 0x1)
    buf = bytearray(4)
    io_i2c.start()
    num_acks = io_i2c.write(bytearray([control_byte]))
    if num_acks != 1:
        return False, None
    io_i2c.readinto(buf)
    io_i2c.stop()
    
    return True, int.from_bytes(buf, "big")

def eeprom_write_int(mem_addr: MemAddr, int_value: int):
    # https://ww1.microchip.com/downloads/en/DeviceDoc/20002213B.pdf
    control_byte = (0xA0 + (mem_addr.block << 1) + 0x0)
    bytes_to_write = [control_byte, mem_addr.addr]
    int_value_as_bytes = list(int_value.to_bytes(4, "big"))
    bytes_to_write.extend(int_value_as_bytes)
    
    io_i2c.start()
    num_acks = io_i2c.write(bytearray(bytes_to_write))
    if num_acks != (2 + 4):  # 1 ack control byte, 1 ack word address, 4 acks 4 bytes of data
        return False
    io_i2c.stop()
    
    utime.sleep_ms(5) # Allow EEPROM time to write
    return True

################
# Key tracking #
################

pressed = []
for row in range(0, len(KEYMAP_L0)):
    pressed_row = []
    for col in range(0, len(KEYMAP_L0[row])):
        pressed_row.append(False)
    pressed.append(pressed_row)
num_keys_down = 0
control_down = False

# Read in the keypress 'odometer' reading from EEPROM and store in shared variable
success, value = eeprom_read_int(MEM_ADDR_TOTAL_KEYRESSES)
if success:
    print("EEPROM successful read (total_keypresses):", value)
    sv_total_keypresses.set_value(value)
else:
    print("Failed to read total_keypresses from EEPROM", success)

# Read in the keypress 'tripmeter' reading from EEPROM and store in shared variable
success, value = eeprom_read_int(MEM_ADDR_KEYPRESS_TRIPMETER)
if success:
    print("EEPROM successful read (keypress_tripmeter):", value)
    sv_keypress_tripmeter.set_value(value)
else:
    print("Failed to read keypress_tripmeter from EEPROM", success)
    
#################
# Core 2 Thread #
#################
# - Runs completely separate from the main (Core 1) thread
# - Responsible for:
#   - Updating contents of OLED screen
#   - Writing and reading from EEPROM


def core_two_thread():
    # This variable is global because it is written to by core 1, and written to/read by core 2
    # Use keypresses_lock when accessing.
    #global sv_total_keypresses, sv_keypress_tripmeter, sv_show_tripmeter
    
    # Apparently gc.collect() causes this module to be inaccessible by this thread?
    # Just re-import it in here
    import utime
    
    #####################
    # Display resources #
    #####################
        
    def draw_centered_text(string, y, size=1, width=128):
        x = int((width / 2) - (len(string) / 2)*6*size)
        oled.text(string, x, y, 1, size=size)
        
    class DisplayState:
        OFF = "OFF"
        BANNER = "BANNER"
        KEYPRESSES = "KEYPRESSES"
        SCREENSAVER = "SCREENSAVER"
    
    current_display_state = DisplayState.BANNER
    last_display_state_transition = utime.time()
    saved_keypresses = None
    last_eeprom_write = utime.time()
    
    #############
    # Main Loop #
    #############
    
    while True:
        try:
            # Enqueue drawing commands based on current display state
            if current_display_state == DisplayState.OFF:
                oled.fill(0)
                
            elif current_display_state in [DisplayState.BANNER, DisplayState.SCREENSAVER]:
                if sv_show_debug_info.get_value():
                    sv_show_debug_info.set_value(False)
                oled.fill(0)
                draw_centered_text("{}'s".format(OWNER), 5, 2)
                draw_centered_text("Brogrammer", 25, 2)
                draw_centered_text("Keyboard", 45, 2)
                
            elif current_display_state == DisplayState.KEYPRESSES:
                # Show the debug screen by default when on the keypress screen
                if not sv_show_debug_info.get_value():
                    sv_show_debug_info.set_value(True)
                    
                oled.fill(0)
                if sv_show_tripmeter.get_value():
                    trip_text = '{:,}'.format(sv_total_keypresses.get_value() - sv_keypress_tripmeter.get_value())
                    total_text = '{:,}'.format(sv_total_keypresses.get_value())
                    size = 2 if (len(trip_text)*12 < 128 and len(total_text)*12 < 128) else 1
                    draw_centered_text(trip_text, 12, size)
                    draw_centered_text(total_text, 32, size)
                else:
                    text = '{:,}'.format(sv_total_keypresses.get_value())
                    size = 2 if (len(text)*12 < 128) else 1
                    draw_centered_text(text, 24, size)
            
            if sv_show_debug_info.get_value():
                matrix_scans_per_sec = sv_matrix_scans_per_sec.get_value()
                oled.rect(0, 54, 128, 10, 0, fill=True)
                oled.text("Mem", 0, 56, 1)
                oled.rect(20, 56, 50, 8, 1, fill=False)
                percent_mem_used = (gc.mem_alloc() / (gc.mem_alloc() + gc.mem_free()))
                oled.rect(20, 56, 50 * percent_mem_used, 8, 1, fill=True)
                if matrix_scans_per_sec:
                    oled.text("Scans", 75, 56, 1)
                    oled.text(str(matrix_scans_per_sec), 110, 56, 1)
                
            # Apply enqueued drawing commands whatever 
            oled.show()
            
            # Handle display state transitions
            now = utime.time()
            new_state = None
            sec_since_last_keypress_for_screensaver = 60
            if current_display_state == DisplayState.BANNER and (now - last_display_state_transition) > 3:
                new_state = DisplayState.KEYPRESSES
            if sv_last_keypress.get_value():
                sec_since_last_keypress = now - sv_last_keypress.get_value()
                if sec_since_last_keypress > sec_since_last_keypress_for_screensaver and current_display_state != DisplayState.SCREENSAVER:
                    new_state = DisplayState.SCREENSAVER
                elif sec_since_last_keypress <= sec_since_last_keypress_for_screensaver and current_display_state == DisplayState.SCREENSAVER:
                    new_state = DisplayState.KEYPRESSES
            if new_state:
                print("Transitioning from", current_display_state, "to", new_state)
                last_display_state_transition = utime.time()
                last_display_state = current_display_state
                current_display_state = new_state
                
            # Every x minutes, write the keypress information to EEPROM (if no other code has forced it)
            x = 5
            if sv_force_eeprom_write.get_value() or (now - last_eeprom_write) > x * 60:
                # Acquire the io_i2c_lock before attempting to communicate with EEPROM,
                # since the I2C bus is shared by the first core (woops, bad design) to communicate
                # with the I/O expander.
                writes = [
                    (MEM_ADDR_TOTAL_KEYRESSES, sv_total_keypresses.get_value()),
                    (MEM_ADDR_KEYPRESS_TRIPMETER, sv_keypress_tripmeter.get_value()),
                ]
                for write in writes:
                    # Try to hold the lock as short as possible (to let core 1 have precendence in
                    # reading the I/O expander)
                    with io_i2c_lock:
                        success = eeprom_write_int(write[0], write[1])
                    if success:
                        print("Wrote to EEPROM")
                    else:
                        print("Failed to write to EEPROM")
                
                sv_force_eeprom_write.set_value(False)
                last_eeprom_write = utime.time()
            
            # Shift the hue of the light strip
            new_hue = sv_hue.get_value() + 0.001
            if new_hue > 1:
                sv_hue.set_value(0)
            else:
                sv_hue.set_value(new_hue)
                
            # Display the current colors of the light strip
            set_all_pixels(colorsys.hls_to_rgb(sv_hue.get_value(), sv_lightness.get_value(), sv_saturation.get_value()))
            show_pixels()
            
        except Exception as e:
            print("Exception:", e)
            raise

_thread.start_new_thread(core_two_thread, ())

#################
# Core 1 Thread #
#################
# - Dedicated to key matrix scanning in a tight loop

matrix_scans = 0
total_matrix_scans = 0
matrix_scan_time = utime.time()
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
                    pin.value(True)
                else:
                    pin.value(False)
            
            # Acquire the io_i2c_lock before attempting to read values from the I/O expander,
            # since the I2C bus is shared by the second core (woops, bad design) to communicate
            # with the EEPROM. 
            with io_i2c_lock:
                col_values = mcp.gpio
            
            # Swap order of upper byte and lower byte
            col_values_adjusted = (0xFF00 & (col_values << 8)) + (0x00FF & (col_values >> 8))
            for col in range(0, len(COL_PINS)):
                pin = COL_PINS[col]
                # Col 0 is a direct GPIO pin
                if col == 0:
                    value = pin.value()
                # The rest are routed through the IO expander
                else:
                    # Get the "bit index" (which bit in the two-byte IO-read does the current column correspond to?)
                    # See the mcp23107.py module for what _pin and _port._port mean
                    bit_index = pin._pin + 8 * pin._port._port
                    value = (col_values & (1 << bit_index)) >> bit_index
                
                # They key is down, and was not previously detected as down
                if value == 0 and not pressed[row][col]:
                    if current_layer == 0:
                        key_type, key = KEYMAP_L0[row][col]
                    elif current_layer == 1:
                        key_type, key = KEYMAP_L1[row][col]

                    print("Key type:", key_type)

                    if key_type == Keycode:
                        print("{:x} at {},{} down".format(key, row, col))
                        if key == Keycode.L1:
                            current_layer = 1
                        else:
                            if key in [Keycode.LEFT_CONTROL, Keycode.RIGHT_CONTROL]:
                                control_down = True
                                print("Control down")
                            try:
                                kbd.press(key)
                                pass
                            except Exception as e:
                                print(e)
                    elif key_type == ConsumerControl:
                        print("Sending ConsumerControl code", key)
                        consumer_control.send(key)
                    elif key_type == Function:
                        if key == "show-tripmeter":
                            print("Toggling show tripmeter")
                            # If the current tripmeter value is None, set it to the current total keypresses
                            if not sv_keypress_tripmeter.get_value():
                                sv_keypress_tripmeter.set_value(sv_total_keypresses.get_value() + 1) # Account for current press
                            
                            # Signal to core two to display the tripmeter
                            sv_show_tripmeter.set_value(not sv_show_tripmeter.get_value())
                            
                            # Force EEPROM write (for better posterity)
                            sv_force_eeprom_write.set_value(True)
                        elif key == "reset-tripmeter":
                            print("Resetting tripmeter")
                            sv_keypress_tripmeter.set_value(sv_total_keypresses.get_value() + 1) # Account for current press
                            
                            # Force EEPROM write (for better posterity)
                            sv_force_eeprom_write.set_value(True)
                        elif key == "reset-total-keypresses":
                            if control_down:
                                print("Resetting total keypresses")
                                # Set the value to 0 and force write it to EEPROM
                                sv_total_keypresses.set_value(-1) # Account for current press
                                sv_keypress_tripmeter.set_value(0)
                                sv_force_eeprom_write.set_value(True)
                        elif key == "show-debug":
                            print("Toggling show debug")
                            # Signal to core two to display the debug info
                            sv_show_debug_info.set_value(not sv_show_debug_info.get_value())
                    else:
                        print("Unassigned key at {},{} down".format(key, row, col))

                    pressed[row][col] = True
                    sv_total_keypresses.set_value(sv_total_keypresses.get_value() + 1)
                    num_keys_down += 1

                # The key is released, and was previously detected as down
                elif value == 1 and pressed[row][col]:
                    if current_layer == 0:
                        key_type, key = KEYMAP_L0[row][col]
                    elif current_layer == 1:
                        key_type, key = KEYMAP_L1[row][col]
                       
                    if key == Keycode.L1:
                        current_layer = 0
                    else:
                        if key_type == Keycode:
                            kbd.release(key)
                            print("{:x} at {},{} released".format(key, row, col))
                            if key in [Keycode.LEFT_CONTROL, Keycode.RIGHT_CONTROL]:
                                control_down = False
                                print("Control up")
                        elif key_type == Function:
                            print("Function [{}] at {},{} released".format(key, row, col))
                        else:
                            print("Unassigned key at {},{} released".format(key, row, col))
                    pressed[row][col] = False
                    num_keys_down -= 1
                    sv_last_keypress.set_value(utime.time())
    except OSError:
        pass

    # If no keys are down, default to a known good state
    if num_keys_down == 0:
        kbd.release_all()
        current_layer = 0
    
    # Determine the number of key matrix scans per second, pass that to the second core to be displayed.
    matrix_scans += 1
    total_matrix_scans += 1
    now = utime.time()
    if (now - matrix_scan_time) >= 1:
        sv_matrix_scans_per_sec.set_value(matrix_scans)
        print(matrix_scans, "matrix scans per sec (memalloc", gc.mem_alloc(), "memfree", gc.mem_free(), ")")
        matrix_scans = 0
        matrix_scan_time = utime.time()
        
    # Handle rotary knob movement
    # See https://en.wikipedia.org/wiki/Rotary_encoder#/media/File:Incremental_directional_encoder.gif
    # If the rotary values are the same, the next time they differ, it indicates which direction the knob
    # is spinning in.
    # In order to support the user spinning the knob quickly the change the volume by large amounts,
    # break into a small loop (away from the main matrix scanning loop) and scan the knob GPIO pins rapidly.
    # The idea is that the user likely won't be typing and spinning the knob at the same time, so it is
    # acceptable to defer from scanning the matrix for 100ms.

    rotary_a_value = rotary_a.value()
    rotary_b_value = rotary_b.value()

    if (last_rotary_a, last_rotary_b, rotary_a_value, rotary_b_value) in [
            (0, 0, 0, 1),
            (0, 0, 1, 0),
            (1, 1, 0, 1),
            (1, 1, 1, 0),
        ]:
        last_rotation = utime.time()
        print("transition")
        while utime.time() - last_rotation < 0.1:
            if last_rotary_a == 0 and last_rotary_b == 0 and rotary_a_value == 0 and rotary_b_value == 1:
                utime.sleep_ms(3)
                rotary_a_value = rotary_a.value()
                rotary_b_value = rotary_b.value()
                if last_rotary_a == 0 and last_rotary_b == 0 and rotary_a_value == 0 and rotary_b_value == 1:
                    print("Knob turned CW, volume incrementing")
                    consumer_control.send(ConsumerControl.VOLUME_INCREMENT)
                    last_rotation = utime.time()
            elif last_rotary_a == 0 and last_rotary_b == 0 and rotary_a_value == 1 and rotary_b_value == 0:
                utime.sleep_ms(3)
                rotary_a_value = rotary_a.value()
                rotary_b_value = rotary_b.value()
                if last_rotary_a == 0 and last_rotary_b == 0 and rotary_a_value == 1 and rotary_b_value == 0:
                    print("Knob turned CCW, volume decrementing")
                    consumer_control.send(ConsumerControl.VOLUME_DECREMENT)
                    last_rotation = utime.time()
            elif last_rotary_a == 1 and last_rotary_b == 1 and rotary_a_value == 0 and rotary_b_value == 1:
                utime.sleep_ms(3)
                rotary_a_value = rotary_a.value()
                rotary_b_value = rotary_b.value()
                if last_rotary_a == 1 and last_rotary_b == 1 and rotary_a_value == 0 and rotary_b_value == 1:
                    print("Knob turned CCW, volume decrementing")
                    consumer_control.send(ConsumerControl.VOLUME_DECREMENT)
                    last_rotation = utime.time()
            elif last_rotary_a == 1 and last_rotary_b == 1 and rotary_a_value == 1 and rotary_b_value == 0:
                utime.sleep_ms(3)
                rotary_a_value = rotary_a.value()
                rotary_b_value = rotary_b.value()
                if last_rotary_a == 1 and last_rotary_b == 1 and rotary_a_value == 1 and rotary_b_value == 0:
                    print("Knob turned CW, volume incrementing")
                    consumer_control.send(ConsumerControl.VOLUME_INCREMENT)
                    last_rotation = utime.time()

            last_rotary_a = rotary_a_value
            last_rotary_b = rotary_b_value
            rotary_a_value = rotary_a.value()
            rotary_b_value = rotary_b.value()
            gc.collect()

    last_rotary_a = rotary_a_value
    last_rotary_b = rotary_b_value
        
    # Running garbage collection manually every matrix scan is crucial.
    # I believe something within the mcp23017 module (or red herring) has a "leak" that,
    # without calling this, will cause memory to be allocated until the Pico crashes.
    # The crash is difficult-to-identify as it's mostly silent.
    gc.collect()
