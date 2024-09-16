#
# module_dc_controller.py
#
# Code for DC controller module for CBUS
#
# Very loosely based on John Purbrick and Martin Da Costa's (MERG) design of 2021,
# but essentially a complete rewrite using Mictopython and uasyncio
# Also based on Duncan Greenwood's CBUS library, but CBUS is not integrated in so far.
#
# (c) Ian Blair 3rd. April 2024
#

import uasyncio as asyncio
from machine import Pin,Timer,UART,DAC,ADC

import aiorepl
import cbus
import cbusconfig
import cbusdefs
import cbusmodule
import mcp2515

from utime import sleep_ms, sleep
import pindefs_DC_Controller_ESP32 as pindefs
import dc_controller_defs as defs
import throttle
import logger

#TestPin=Pin(pindefs.PIN_SND_TXD,Pin.OUT)
timer=Timer()
log=logger.logger()
              

class dc_controller_mymodule(cbusmodule.cbusmodule):
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
        last_bemf=0
        
    # Filter calculates instantaneous output value based on mode, phase, and start_level
    def filter_calc(self, mode, phase, throttle_level, start_level):
        switching_phase = int(throttle_level*defs.MAX_PHASE/defs.MAX_THROTTLE_LEVEL)
        if (mode == defs.MODE_DIRECT):
            return_value = input_level
        elif ((mode == defs.MODE_TRIANGLE) or mode == (defs.MODE_TRIANGLE_BEMF)):
            if (phase < switching_phase):
                return_value = min(start_level + int(phase/defs.MAX_PHASE),defs.MAX_OP_LEVEL)
            else:
                return_value = min(start_level + int((phase-switching_phase)/defs.MAX_PHASE), defs.MAX_OP_LEVEL)
        return(return_value)
        
    # This calculates the overall throttle level based on the pot setting bemf measurement and selected mode    
    def calculate_throttle(self, mode, requested_speed, bemf_speed):
        # By default or if mode is direct, output = input
        # (level matching TBA)
        output_level=requested_speed
        if (mode == defs.MODE_TRIANGLE):
            # This to be revisited later:-
            # Martin Da Costa ignores 40 percent limit
            # but applies feedback in inverse proportion to requested speed instead
            if ((requested_speed>=defs.FORTY_PERCENT_THROTTLE)or(bemf_speed>defs.MAX_BEMF_LEVEL)):
                previous_bemf=bemf_speed
            else:
                error_level = (int((requested_speed*defs.INP_SCALE-(previous_bemf+bemf_speed))*defs.ERROR_SCALE)/defs.ERROR_DIV)
                if(error_level<0):
                    pass
                elif((requested_speed + error_level)>defs.MAX_OP_LEVEL):
                    output_level = defs.MAX_OP_LEVEL
                elif((requested_speed + error_level)>0):
                    output_level = requested_speed + error_level
                    output_level=requested_speed
            previous_bemf=bemf_speed
        return(output_level)

    def initialise(self):
        
        # ***
        # *** CBUS part, bare minimum module init
        # *** Can be omitted for standalone use, TBD
        # ***

        self.cbus = cbus.cbus(
            mcp2515.mcp2515(),
            cbusconfig.cbusconfig(storage_type=cbusconfig.CONFIG_TYPE_FILES),
        )

        # ** change the module name and ID if desired

        self.module_id = 103
        self.module_name = bytes('PYCO   ', 'ascii')
        self.module_params = [
            20,
            cbusdefs.MANU_MERG,
            0,
            self.module_id,
            self.cbus.config.num_events,
            self.cbus.config.num_evs,
            self.cbus.config.num_nvs,
            1,
            7,
            0,
            cbusdefs.PB_CAN,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]

        # change the CBUS switch and LED pin numbers if desired

        self.cbus.set_leds(21, 20)
        self.cbus.set_switch(22)
        self.cbus.set_name(self.module_name)
        self.cbus.set_params(self.module_params)
        self.cbus.set_event_handler(self.event_handler)
        self.cbus.set_received_message_handler(self.received_message_handler)
        self.cbus.set_sent_message_handler(self.sent_message_handler)

        self.cbus.begin()

        # ***
        # ***
        # Controller part
        # instantiate pins
        t0dac = DAC(pindefs.PIN_DAC0)
        t1dac = DAC(pindefs.PIN_DAC1)
        bemf0adc = ADC(pindefs.PIN_BEMF0,atten=ADC.ATTN_11DB)
        bemf1adc = ADC(pindefs.PIN_BEMF1,atten=ADC.ATTN_11DB)
        self._potadc = ADC(pindefs.PIN_POT,atten=ADC.ATTN_11DB)
        blnk0pin = Pin(pindefs.PIN_BLNK0)
        blnk1pin = Pin(pindefs.PIN_BLNK1)
        self._dirpin = Pin(pindefs.PIN_DIR)
        
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

        while True:
            requested_level = self._potadc.read_u16()
            throttle_value=0
            start_level=0 # exact value of start_level is TBD
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
                    if (phase == defs.POT_PHASE):
                        requested_level = self._potadc.read_u16()
                        
                    elif (phase == defs.BLANK_PHASE):
                       self.output_throttle.set_blanking()
                       blanking_enabled = True

                    elif (phase == defs.BEMF_PHASE):
                        if (blanking_enabled):
                           bemf_level= self.output_throttle.read_bemf()
                           blanking_enabled = False
                       
                    # At end of each cycle recalculate throttle values
                    elif (phase == defs.LAST_PHASE):
                        self.output_throttle.clear_blanking()
                        throttle_value = self.calculate_throttle(defs.MODE_TRIANGLE,requested_level,bemf_level)
                
                    
                    # Regardless of above actions set output value according top hase
                    # Note that output will only be seen when blanking is not enabled.
                    output_value=self.filter_calc(defs.MODE_TRIANGLE,phase,throttle_value,start_level)
                    self.output_throttle.write_output(output_value)
                    # Then wait for next loop
                    # For testing - for normal operation replaced by separate async task
                    #await asyncio.sleep_us(100)
            else:
                # Otherwise reverse direction
                self.last_direction = direction                
                set_throttle(direction)                
                

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
        self.tb = asyncio.create_task(self.throttle_coro())
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
