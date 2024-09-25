#
# dc_controller_defs.py
#
# DC controller definitions
#
# (c) Ian Blair 3rd. April 2024
#
# For license and attributions see associated readme file
#

#Phase values
#MAX_PHASE=const(1024)
#POT_PHASE=const(32)
#BLANK_PHASE=const(896)
#BEMF_PHASE=const(960)
#LAST_PHASE=const(MAX_PHASE-1)                

MAX_PHASE=const(16)
POT_PHASE=const(3)
BLANK_PHASE=const(14)
LAST_PHASE=const(MAX_PHASE-1)

# These are to be set up...
#pot_scale= x
#bemf_scale = y
#output_scale = z

# Levels and scale factors
MIN_REQUESTED_LEVEL= const(10)
MAX_THROTTLE_LEVEL=const(4095)
FORTY_PERCENT_THROTTLE=int(MAX_THROTTLE_LEVEL*40/100)
MAX_OP_LEVEL=const(255)
MAX_BEMF_LEVEL=const(1000)
ERROR_SCALE=const(8)
INP_SCALE=const(8) #Equivalent of 8 used for Ardiono. Divide by 2 for ADC reference 2.45V,
#and a further 4 for input range (12 bits, not 10)

# Modes
MODE_DIRECT=const(0)
MODE_TRIANGLE=const(1)
