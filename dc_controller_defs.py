#
# dc_controller_defs.py
#
# DC controller definitions
#
# (c) Ian Blair 3rd. April 2024
#

#Phase values
MAX_PHASE=const(1024)
    
POT_PHASE=const(32)
BLANK_PHASE=const(896)
BEMF_PHASE=const(960)
LAST_PHASE=const(MAX_PHASE-1)                

# These are to be set up...
#pot_scale= x
#bemf_scale = y
#output_scale = z

# Levels and scale factors
MIN_REQUESTED_LEVEL= const(10)
MAX_THROTTLE_LEVEL=const(256)
FORTY_PERCENT_THROTTLE=int(MAX_THROTTLE_LEVEL*40/100)
MAX_OP_LEVEL=const(256)
MIN_BEMF_LEVEL=const(10)
ERROR_SCALE=const(8)

# Modes
MODE_DIRECT=const(0)
MODE_TRIANGLE=const(1)
