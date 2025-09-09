import board
from ads122C04 import ADS122
import time

# This example demonstrates how to use the ADS122 using single-shot conversion mode 

adc = ADS122(board.I2C())

adc.set_channel(ADS122.CHANNEL_AIN0_AIN1)
adc.set_gain(ADS122.GAIN_64X)

while True:
    result = adc.read_single_shot()
    result_mv = adc.to_mv(result)
    print(f"({result_mv:> 8.4f})")
    time.sleep(0.1)      