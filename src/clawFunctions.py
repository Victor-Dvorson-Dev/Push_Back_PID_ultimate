#region VEXcode Generated Robot Configuration
from vex import *
import random as urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
digital_out_a = DigitalOut(brain.three_wire_port.a)
digital_out_b = DigitalOut(brain.three_wire_port.b)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode V5 Python Project
# 
# ------------------------------------------

# Library imports
from vex import *

# Begin project code

"""
function to flip the claw. Doesnt take any input and optionally returns the position of the claw after flipping it.
You can also access the position of the claw by using the global variable clawPosition. 0 = starting position, 1 = flipped position
"""
clawPosition = 0
def flip_claw():
    global clawPosition
    if digital_out_b.value() == False:
        digital_out_b.set(True)
        clawPosition = 1
    else:
        digital_out_b.set(False)
        clawPosition = 0

    #so kawaii :3 <3
    brain.screen.clear_line(1)
    brain.screen.set_cursor(1, 1)
    brain.screen.print("so kawaii :3 <3 "+str(clawPosition))

    return clawPosition


toggleClaw = False
toggleFlip = False
while True:
    if controller_1.buttonR1.pressing():
        if toggleFlip == False:
            flip_claw()
        toggleFlip = True
    elif toggleFlip == True:
        toggleFlip = False
    

    #Opens an closes claw
    if controller_1.buttonY.pressing():
        if toggleClaw == False:
            if digital_out_a.value() == False:
                digital_out_a.set(True)
            else:
                digital_out_a.set(False)
        toggleClaw = True
    elif toggleClaw == True:
        toggleClaw = False

