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
from machine import Pin,SPI,Timer,UART

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
        throttle0.update_output(0)
        throttle1_update_output(0)
        # The set throttles according to direction
        # Output throttle will drive trains
        # Return throttle will remain at zero for return rail. 
    if (forward_not_backwards == True):
        output_throttle = throttle0
        return_throttle = throttle1
    else:
        output_throttle = throttle1
        return_throttle = throttle0
    # delete stored bemf_level
    last_bemf=0
        
    # Filter calculates instantaneous output value based on mode, phase, and start_level
    def filter(mode, phase, throttle_level, start_level):
        switching_phase = int(throttle_level*defs.MAX_PHASE/defs.MAX_THROTTLE_LEVEL)
        if (mode == defs.MODE_DIRECT):
            return_value = input_level
        elif ((mode == defs.MODE_TRIANGLE) or mode == (defs.MODE_TRIANGLE_BEMF)):
            if (phase < switching_phase):
                return_value = min(start_level + phase/MAX_PHASE,MAX_OP_LEVEL)
            else:
                return_value = min(start_level + (phase-switching_phase), MAX_OP_LEVEL)
        return(return_value)
        
    # This calculates the overall throttle level based on the pot setting bemf measurement and selected mode    
    def calculate_throttle(mode, requested_speed, bemf_speed)
        if (mode == defs.MODE_DIRECT):
            output_level = requested_speed
        elif (mode == defs.TRIANGLE_BEMF):
            if ((requested_speed>=FORTY_PERCENT_THROTTLE)||(bemf_speed>MIN_BEMF_LEVEL)):
                previous_bemf=bemf_speed
            else:
                error_level = requested_speed*_inp_scale-(previous_bemf+bemf_speed)*defs.ERROR_SCALE
                if(error_level<0):
                    output_level=requested_speed
        return(output_value)


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
        # Controller part
        # Instantiate throttles, ine for each output
               # ***
        # Controller part
        # instantiate pins
        t0dac = DAC(pindefs.PIN_DAC0)
        t1dac = DAC(pindefs.PIN_DAC1)
        bemf0adc = ADC(pindefs.PIN_BEMF0)
        bemf1adc = ADC(pindefs.PIN_BEMF1)
        potadc = bemf0adc
        blnk0pin = Pin(pindefs.PIN_BLNK0)
        blnk1pin = Pin(pindefs.PIN_BLNK1)
        dirpin = Pin(pindefs.PIN_DIR)
        
        # Instantiate throttles, using instantiated pins, one for each output
        throttle0 = throttle(t0dac, bemf0adc, blnk0pin)
        throttle1 = throttle(t1dac, bemf1adc, blnk1pin)        
        
        direction = dirpin.value()
        self.set_throttle(direction)
        last_direction = direction
        
    # ***
    # *** coroutines that run in parallel
    # ***
    # *** Throttles coro
    async def throttles_coro(self) -> None:
        self.logger.log('throttles_coro start')

        while True:
            requested_level = potadc.read_u16()
            # only act on direction switch when requested_level is below minimum threshold
            if (requested_level<min_requested_level)
        direction = dirpin.value()
            else:
                direction = last_direction
            blanking_enabled = False
            
            start_level=0 # exact value of start_level is TBD
                
            if (direction == last_direction):
                
                # Main waveform loop
                # Performs various actions according to phase value
                for phase in range max_phase:
                    
                    await asyncio._timer_synchronisation.wait()
                    # Perform required actions on particular phases
                    if (phase == pot_phase):
                        requested_level = potadc.read_u16()
                        
                    elif (phase == blank_phase):
                       set_blanking()
                       blanking_enabled = True

                    elif (phase == bemf_phase):
                        if (blanking_enabled):
                           bemf_level== potbemf.read_u16()
                           blanking_enabled = False
                       
                    # At end of each cycle recalculate throttle values
                    elif (phase == last_phase):
                        clear_blanking()
                        throttle_value = output_throttle.calculate_throttle(throttle_mode,requested_level,bemf_level)
                
                    
                    # Regardless of above actions set output value according top hase
                    # Note that output will only be seen when blanking is not enabled.
                    output_value=filter(mode,phase,throttle_value,start_value)
                    current_throttle.update_output(output_value)
                    # Then wait for next loop
                    # For testing - for normal operation replaced by separate async task
                    #await asyncio.sleep_us(100)
            else:
                # Otherwise reverse direction

                last_direction = direction                
                set_throttle(direction)
                

    # *** user module application task - like Arduino loop()
    async def module_main_loop_coro(self) -> None:
        self.logger.log('main loop coroutine start')
        while True:
            await asyncio.sleep_us(100)
            asyncio._timer_synchronisation.set()

    # ***
    # *** module main entry point - like Arduino setup()
    # ***

    async def run(self) -> None:
        # self.logger.log('run start')

        # start coroutines
        self.tb = asyncio.create_task(self.throttle_coro())
        self.tm = asyncio.create_task(self.module_main_loop_coro())

        repl = asyncio.create_task(aiorepl.task(globals()))

        self.logger.log('module startup complete')
        await asyncio.gather(repl)


# create the module object and run it
mod = dc_controller_mymodule()
mod.initialise()
asyncio.run(mod.run())

# the asyncio scheduler is now in control
# no code after this line is executed

print('*** application has ended ***')
