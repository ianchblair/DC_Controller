#
# dc_controller.py
#
# Code for standalone DC controller
#
# Very loosely based on John Purbrick and Martin Da Costa's (MERG) design of 2021,
# but essentially a complete rewrite using Mictopython and uasyncio
# CBUS interface is omittedd in this version
#
# (c) Ian Blair 6th. Sept 2024
#
# For license and attributions see associated readme file
#

import uasyncio as asyncio
from machine import Pin,Timer,UART,DAC,ADC

import aiorepl

from utime import sleep_us, sleep_ms, sleep
import pindefs_dc_controller_esp32 as pindefs
import dc_controller_defs as defs
import throttle
#import logger

#TestPin=Pin(pindefs.PIN_SND_TXD,Pin.OUT)
#timer=Timer()
#log=logger.logger()
              

class dc_controller_mymodule():
    def __init__(self):
        super().__init__()

        
    def set_throttle(self,forward_not_backwards):
        # Set both outputs to zero before setting/swapping
        self.throttle0.write_output(0)
        self.throttle1.write_output(0)
        # The set throttles according to direction
        # Output throttle will drive trains
        # Return throttle will remain at zero for return rail. 
        if (forward_not_backwards == True):
            self.output_throttle = self.throttle0
            self.return_throttle = self.throttle1
        else:
            self.output_throttle = self.throttle1
            self.return_throttle = self.throttle0
    # delete stored bemf_level
        self.last_bemf=0
        
    # Filter calculates instantaneous output value based on mode, and phase
    def filter_calc(self, mode, phase, throttle_level):
        # Offset the DC according to the throttle vale
        dc_offset=int(throttle_level*defs.MAX_OP_LEVEL/defs.MAX_THROTTLE_LEVEL)
        switching_phase = int(throttle_level*defs.MAX_PHASE/defs.MAX_THROTTLE_LEVEL)
        if (mode == defs.MODE_DIRECT):
            return_value = dc_offset
        elif ((mode == defs.MODE_TRIANGLE) or mode == (defs.MODE_TRIANGLE_BEMF)):
            if (phase < switching_phase):
                triangle_value = min(int((phase*defs.MAX_OP_LEVEL)/defs.MAX_PHASE),defs.MAX_OP_LEVEL)
            else:
                height = int((switching_phase*defs.MAX_OP_LEVEL)/defs.MAX_PHASE)
                triangle_value = min(height + int(((switching_phase-phase)*defs.MAX_OP_LEVEL)/defs.MAX_PHASE), defs.MAX_OP_LEVEL)
            if (triangle_value < 0):
                triangle_value = 0
            # Only use triangle wave for outputs below 50%
            if (dc_offset<defs.MAX_OP_LEVEL/2):    
                return_value = int(triangle_value*(defs.MAX_OP_LEVEL-(2*dc_offset))/defs.MAX_OP_LEVEL) + dc_offset
            else:
                return_value = dc_offset

        # Extra check to imit output to range of DAC, where offset puts it out of range
        if (return_value < 0):
            return_value = 0
        if (return_value > defs.MAX_OP_LEVEL):
            return_value = defs.MAX_OP_LEVEL
            
        return(return_value)
        
    # This calculates the overall throttle level based on the pot setting bemf measurement and selected mode    
    def calculate_throttle(self, mode, requested_speed, bemf_speed):
        # By default or if mode is direct, output = input
        # Input levels arefrom ADCs, 0..4095 range
        # (level matching TBA)
        output_level=requested_speed
        if ((mode == defs.MODE_TRIANGLE) and (bemf_speed < defs.MAX_BEMF_LEVEL)):
            ## This to be revisited later:-
            ## Martin Da Costa ignores 40 percent limit
            ## but applies feedback in inverse proportion to requested speed instead
            ##if ((requested_speed>=defs.FORTY_PERCENT_THROTTLE)or(bemf_speed>defs.MAX_BEMF_LEVEL)):
            ##    self.last_bemf=bemf_speed
            ##else:
            error_correction = ((self.last_bemf+bemf_speed)*defs.ERROR_SCALE)
            error_level = requested_speed-error_correction
            scaled_error_level = int(error_level*(defs.MAX_THROTTLE_LEVEL-requested_speed)/defs.MAX_THROTTLE_LEVEL)
            if(error_level>=0):
                if((requested_speed + scaled_error_level)>defs.MAX_THROTTLE_LEVEL):
                    output_level = defs.MAX_THROTTLE_LEVEL
                elif((requested_speed + scaled_error_level)>0):
                    output_level = requested_speed + scaled_error_level
                else:
                    output_level=requested_speed
        # save bemf measuremant for next cycle
        self.last_bemf=bemf_speed
        # return calculated throttle value
        return(output_level)


    def initialise(self):
        
        # ***
        # Controller part
        # instantiate pins
        t0dac = DAC(pindefs.PIN_DAC0)
        t1dac = DAC(pindefs.PIN_DAC1)
        bemf0adc = ADC(pindefs.PIN_BEMF0,atten=ADC.ATTN_11DB)
        bemf1adc = ADC(pindefs.PIN_BEMF1,atten=ADC.ATTN_11DB)
        self._potadc = ADC(pindefs.PIN_POT,atten=ADC.ATTN_11DB)
        blnk0pin = Pin(pindefs.PIN_BLNK0, Pin.OUT)
        blnk1pin = Pin(pindefs.PIN_BLNK1, Pin.OUT)
        self._dirpin = Pin(pindefs.PIN_DIR, Pin.IN, Pin.PULL_UP)
        
        # Instantiate throttles, using instantiated pins, one for each output
        self.throttle0 = throttle.throttle(t0dac, bemf0adc, blnk0pin)
        self.throttle1 = throttle.throttle(t1dac, bemf1adc, blnk1pin)        
        
        direction = self._dirpin.value()
        self.set_throttle(direction)
        self.last_direction = direction
        self._timer_synchronisation=asyncio.ThreadSafeFlag()
        
    # ***
    # *** coroutines that run in parallel
    # ***
    # *** Throttles coro
    async def throttles_coro(self) -> None:
#        self.logger.log('throttles_coro start')
        self.output_throttle.clear_blanking()
        self.return_throttle.clear_blanking()
        # default these to zero until assigned further down...
        throttle_value=0
        requested_level=0
        
        while True:
            # only act on direction switch when requested_level is below minimum threshold
            if (requested_level<defs.MIN_REQUESTED_LEVEL):
                direction = self._dirpin.value()
            else:
                direction = self.last_direction
            blanking_enabled = False
                
            if (direction == self.last_direction):
                
                # Main waveform loop
                # Performs various actions according to phase value
                for phase in range(defs.MAX_PHASE):
                    
                    await self._timer_synchronisation.wait()
                    # Perform required actions on particular phases
                    # Start all cycles with blanking off on both throttles
                    if (phase == 0):
                       self.output_throttle.clear_blanking()
                       
                    elif (phase == defs.POT_PHASE):
                        requested_level = self._potadc.read_u16()
                        
                    elif (phase == defs.BLANK_PHASE):
                       self.output_throttle.set_blanking()
                       blanking_enabled = True
                       
                    # At end of each cycle recalculate throttle values
                    elif (phase == defs.LAST_PHASE):
                        if (blanking_enabled):
                           bemf_level= self.output_throttle.read_bemf()
                           #bemf_level= 0
                           blanking_enabled = False
                        throttle_value = self.calculate_throttle(defs.MODE_TRIANGLE,requested_level,bemf_level)
                        #throttle_value=requested_level
                
                    
                    # Regardless of above actions set output value
                    # Note that output will only be seen when blanking is not enabled.
                    #output_sample=self.filter_calc(defs.MODE_TRIANGLE,phase,1000)
                    output_sample=self.filter_calc(defs.MODE_TRIANGLE,phase,throttle_value)
                    self.output_throttle.write_output(output_sample)
                    #self.output_throttle.write_output(requested_level)
                    # Then wait for next loop
                    # For testing - for normal operation replaced by separate async task
                    #await asyncio.sleep_us(100)
            else:
                # Otherwise reverse direction
                self.last_direction = direction                
                self.set_throttle(direction)
                # Reset blanking
                self.output_throttle.clear_blanking()
                self.return_throttle.clear_blanking() 

    # *** user module application task - like Arduino loop()
    async def module_main_loop_coro(self) -> None:
#        self.logger.log('main loop coroutine start')
        while True:
            await asyncio.sleep_ms(1)
            self._timer_synchronisation.set()

    # ***
    # *** module main entry point - like Arduino setup()
    # ***

    async def run(self) -> None:
        # self.logger.log('run start')

        # start coroutines
        self.tb = asyncio.create_task(self.throttles_coro())
        self.tm = asyncio.create_task(self.module_main_loop_coro())

        repl = asyncio.create_task(aiorepl.task(globals()))

#        self.logger.log('module startup complete')
        await asyncio.gather(repl)


# create the module object and run it
mod = dc_controller_mymodule()
mod.initialise()
asyncio.run(mod.run())

# the asyncio scheduler is now in control
# no code after this line is executed

print('*** application has ended ***')
