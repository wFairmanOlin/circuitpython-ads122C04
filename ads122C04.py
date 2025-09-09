from micropython import const
import struct
import time

# standalone commands
_COMMAND_RESET      = const(0b00000110)
_COMMAND_START_SYNC = const(0b00001000)
_COMMAND_POWERDOWN  = const(0b00000010)
_COMMAND_RDATA      = const(0b00010000)

# commands that require parameter register
_COMMAND_RREG       = const(0b00100000)
_COMMAND_WREG       = const(0b01000000)

# parameter registers
_GAIN_REG    = const(0b00000000)
_CHANNEL_REG = const(0b00000000)
_DRDY_REG    = const(0b00001000)

# parameter masks
_GAIN_MASK = const(0b00001110)
_CHANNEL_MASK = const(0b11110000)
_DRDY_MASK = const(0b10000000)

_DRDY_NO_NEW_RESULT = const(0b00000000)    # No new conversion result available
_DRDY_NEW_RESULT_READY = const(0b10000000) # New conversion result ready

class ADS122:
    CHANNEL_AIN0_AIN1 = const(0b00000000)  # Differential P = AIN0, N = AIN1 (default)
    CHANNEL_AIN0_AIN2 = const(0b00010000)  # Differential P = AIN0, N = AIN2
    CHANNEL_AIN1_AIN0 = const(0b00110000)  # Differential P = AIN1, N = AIN0 (default)
    CHANNEL_AIN0 = const(0b10000000)       # Single-ended AIN0
    CHANNEL_AIN1 = const(0b10010000)       # Single-ended AIN1
    CHANNEL_AIN2 = const(0b10100000)       # Single-ended AIN2
    CHANNEL_AIN3 = const(0b10110000)       # Single-ended AIN3
    CHANNEL_SUPPLY = const(0b11010000)   # Mid-supply   P = AVDD/4, N = AVSS/4
    
    GAIN_1X = const(0b0000)   # Gain = 1 (default)
    GAIN_2X = const(0b0010)   # Gain = 2
    GAIN_4X = const(0b0100)   # Gain = 4
    GAIN_8X = const(0b0110)   # Gain = 8
    GAIN_16X = const(0b1000)  # Gain = 16
    GAIN_32X = const(0b1010)  # Gain = 32
    GAIN_64X = const(0b1100)  # Gain = 64
    GAIN_128X = const(0b1110) # Gain = 128
    
    VREF_INTERNAL_MV = 2048 # Internal reference voltage = 2048 mV

    def __init__(self, i2c, address=0b01000000):
        self._i2c = i2c
        self._address = address
        self.reset()

    def _blocking_write(self, data):
        # wait for i2c bus availability
        while not self._i2c.try_lock():
            time.sleep(0.0001)
        self._i2c.writeto(self._address, data)
        self._i2c.unlock()

    def _read_modify_write(self, register, mask, value):
        as_is = self.read_register(register)
        to_be = (as_is & ~mask) | value
        register |= _COMMAND_WREG
        wreg = struct.pack('BB', register, to_be)
        self._blocking_write(wreg)

    def read_register(self, register):
        register |= _COMMAND_RREG
        rreg = struct.pack('B', register)
        self._blocking_write(rreg)
        config = bytearray(1)
        self._i2c.try_lock()
        self._i2c.readfrom_into(self._address, config)
        self._i2c.unlock()
        return config[0]

    def set_channel(self, channel):
        self._read_modify_write(_CHANNEL_REG, _CHANNEL_MASK, channel)
        
    def set_gain(self, gain):
        self._read_modify_write(_GAIN_REG, _GAIN_MASK, gain)
        
    def read_single_shot(self):
        self.start_sync()
        # loop until conversion is complete
        while ((self.read_register(_DRDY_REG) & _DRDY_MASK) == _DRDY_NO_NEW_RESULT):
            time.sleep(0.0001)

        rreg = struct.pack('B', _COMMAND_RDATA) 
        self._blocking_write(rreg)
        data = bytearray(3)
        self._i2c.try_lock()
        self._i2c.readfrom_into(self._address, data)
        self._i2c.unlock()
        value = int.from_bytes(data)
        # value is negative
        if value > (1 << 23):
            value = (value - (1 << 24))
        return value
        
    def to_mv(self, value):
        # read gain
        gain = self.read_register(_GAIN_REG)
        gain &= _GAIN_MASK
        gain = 2 ** (gain >> 1)
        # mV of lsb
        lsb = 2 * self.VREF_INTERNAL_MV / gain / 2**24
        return value * lsb
    
    def reset(self):
        data = struct.pack('B', _COMMAND_RESET)
        self._blocking_write(data)

    def start_sync(self):
        data = struct.pack('B', _COMMAND_START_SYNC)
        self._blocking_write(data)     

    def powerdown(self):
        data = struct.pack('B', _COMMAND_POWERDOWN)
        self._blocking_write(data)