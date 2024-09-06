#
# throttle.py
#
# Code for each DC Controller throttle using asyncio library
#
# (c) Ian Blair 3rd. April 2024
#

import uasyncio as asyncio
from machine import Pin,SPI,Timer,DAC,ADC
from utime import sleep_ms, sleep
import pindefs_ESP32_WROOM as pindefs
import dc_controller_defs
import logger

log=logger.logger()

    # Controller routines
def adc_read(self,adc_instance):
    _adc_value = adc.read_u16()
    return(_adc_value)
        
def dac_write(self,dac_value,dac_instance):
    dac.write(dac_value)

    
class throttle(throttle_pin, bemf_pin, blnk_pin):
    def __init__(self):
        super().__init__()
        self._throttle_pin = throttle_pin
        self._adc_bemf_instance = bemf_pin
        self._dac_instance = self._throttle_pin
        self._blanking = blnk_pin
        phase = 0
        previous_bemf = 0

    def initialise(self):
        # TBD    
            
    def set_blanking(self):
    self._blanking.value(0)

    def clear_blanking(self):
    self._blanking.value(1)
    
    def read_bemf(self):
        return(adc.bemf_instance.adc_read())
        
    def write_output(self,output_level)
        self._dac_instance.dac_write(output_level)
        


 