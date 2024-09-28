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
import pindefs_dc_controller_esp32 as pindefs
import dc_controller_defs

class throttle:    
    #def __init__(self, throttle_pin=Pin(), bemf_pin=Pin(), blnk_pin=Pin()):
    def __init__(self, throttle_pin, bemf_pin, blnk_pin) -> None:
        super().__init__()
        self._throttle_pin = throttle_pin
        self._adc_bemf_instance = bemf_pin
        self._dac_instance = self._throttle_pin
        self._blanking = blnk_pin
            
    # Controller routines
    def adc_read(self,adc_instance):
        _adc_value = adc_instance.read()
        return(_adc_value)
        
    def dac_write(self,dac_value,dac_instance):
        dac_instance.write(dac_value)
        
    def set_blanking(self):
        self._blanking.value(0)

    def clear_blanking(self):
        self._blanking.value(1)
    
    def read_bemf(self):
        return(self._adc_bemf_instance.read_u16())
        
    def write_output(self,output_level):
        self._dac_instance.write(output_level)
        


 