# 
# from machine import Pin, I2C
# import utime
# 
# 
# io_scl = Pin(11, Pin.OUT)
# io_sda = Pin(10)
# 
# io_i2c = I2C(id=1, scl=io_scl, sda=io_sda, freq=100 * 1000)
# 
# print(io_i2c.scan)
# 
# def eeprom_read_int(i2c_addr, block_num, mem_addr):
#     # https://ww1.microchip.com/downloads/en/DeviceDoc/20002213B.pdf
#     try:
#         # First change address
#         control_byte = (0xA0 + (block_num << 1) + 0x0)
#         num_acks = io_i2c.writeto(i2c_addr, bytearray([control_byte, mem_addr]), False)
#         if num_acks != 2:
#             return False
# 
#         # Then read data at the address
#         control_byte = (0xA0 + (block_num << 1) + 0x1)
#         buf = bytearray(4)
#         
#         num_acks = io_i2c.writeto(i2c_addr, bytearray([control_byte]), False)
#         if num_acks != 1:
#             return False
#         io_i2c.readfrom_into(i2c_addr, buf)
#     except OSError:
#         return False, None
#     return True, int.from_bytes(buf, "big")
# 
# def eeprom_write_int(i2c_addr, block_num, mem_addr, int_value):
#     # https://ww1.microchip.com/downloads/en/DeviceDoc/20002213B.pdf
#     control_byte = (0xA0 + (block_num << 1) + 0x0)
#     bytes_to_write = [control_byte, mem_addr]
#     int_value_as_bytes = list(int_value.to_bytes(4, "big"))
#     bytes_to_write.extend(int_value_as_bytes)
#     
#     try:
#         num_acks = io_i2c.writeto(i2c_addr, bytearray(bytes_to_write))
#         if num_acks != 6:
#             return False
#         utime.sleep_ms(10) # Allow EEPROM time to write
#     except OSError:
#         return False
# 
#     return True
# 
# 
# ########
# # TEST #
# ########
# 
# for addr in range(0, 2):
#     success = eeprom_write_int(0x50, 1, addr, addr*10)
#     if success:
#         print("Wrote to EEPROM block 1 address", addr, "value", addr*10)
#     else:
#         print("Failed to write to address", addr)
# 
# 
# for addr in range(0, 2):
#     success, value = eeprom_read_int(0x50, 1, addr)
#     if success:
#         print("Read from EEPROM block", 1, "addr", addr, "value read:", value)
#     else:
#         print("Failed to read EEPROM", success)
#         
#




from machine import Pin, SoftI2C
import utime


io_scl = Pin(11, Pin.OUT)
io_sda = Pin(10)

io_i2c = SoftI2C(io_scl, io_sda, freq=100 * 1000)

def eeprom_read_int(block_num, mem_addr):
    # https://ww1.microchip.com/downloads/en/DeviceDoc/20002213B.pdf
    io_i2c.start()
    control_byte = (0xA0 + (block_num << 1) + 0x0)
    num_acks = io_i2c.write(bytearray([control_byte, mem_addr]))
    if num_acks != 2:
        return False, None
    
    control_byte = (0xA0 + (block_num << 1) + 0x1)
    buf = bytearray(4)
    io_i2c.start()
    num_acks = io_i2c.write(bytearray([control_byte]))
    if num_acks != 1:
        return False, None
    io_i2c.readinto(buf)
    io_i2c.stop()
    
    return True, int.from_bytes(buf, "big")

def eeprom_write_int(block_num, mem_addr, int_value):
    # https://ww1.microchip.com/downloads/en/DeviceDoc/20002213B.pdf
    control_byte = (0xA0 + (block_num << 1) + 0x0)
    bytes_to_write = [control_byte, mem_addr]
    int_value_as_bytes = list(int_value.to_bytes(4, "big"))
    bytes_to_write.extend(int_value_as_bytes)
    
    io_i2c.start()
    num_acks = io_i2c.write(bytearray(bytes_to_write))
    if num_acks != (2 + 4):  # 1 ack control byte, 1 ack word address, 4 acks 4 bytes of data
        return False
    io_i2c.stop()
    
    utime.sleep_ms(5) # Allow EEPROM time to write
    return True
    


########
# TEST #
########

for block in range(0, 8):
    for word in range(0, 16):
        addr = word*4
        success = eeprom_write_int(block, addr, addr*10)
        if success:
            print("Wrote to EEPROM block", block, "address", addr, "value", addr*10)
        else:
            print("Failed to write to address", addr)


for block in range(0, 8):
    for word in range(0, 16):
        addr = word*4
        success, value = eeprom_read_int(block, addr)
        if success:
            if value == addr*10:
                print("Read from EEPROM block", block, "addr", addr, "value:", value, "")
            else:
                print("UNEXPECTED VALUE Read from EEPROM block", block, "addr", addr, "value:", value, "")
        else:
            print("Failed to read EEPROM")
        
        
