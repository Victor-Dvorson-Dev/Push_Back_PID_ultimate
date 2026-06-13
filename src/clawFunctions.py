#region VEXcode Generated Robot Configuration
from vex import *
import random as urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
motor_18 = Motor(Ports.PORT18, GearSetting.RATIO_18_1, False)
controller_1 = Controller(PRIMARY)
digital_out_a = DigitalOut(brain.three_wire_port.a)


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

motor_18.set_velocity(100, PERCENT)

"""
function to flip the claw. Takes in a string that specifies which claw to flip.
top: flips the top claw
bottom: flips the bottom claw
both: flips both claws
"""
topClawPosition = 0
bottomClawPosition = 0

def flip_claw(claw):
    global topClawPosition
    global bottomClawPosition
    
    if claw == "top":
        if topClawPosition + bottomClawPosition %2 == 0:
            motor_18.spin_for(FORWARD, 180, DEGREES)
            topClawPosition += 1
        else:
            motor_18.spin_for(REVERSE, 180, DEGREES)
            topClawPosition -= 1

    elif claw == "bottom":
        if topClawPosition + bottomClawPosition %2 == 0:
            motor_18.spin_for(REVERSE, 180, DEGREES)
            bottomClawPosition -= 1
        else:
            motor_18.spin_for(FORWARD, 180, DEGREES)
            bottomClawPosition += 1

    elif claw == "both":
        if topClawPosition + bottomClawPosition %2 == 0:
            motor_18.spin_for(FORWARD, 360, DEGREES)
            topClawPosition += 1
            bottomClawPosition += 1

        else:
            motor_18.spin_for(REVERSE, 360, DEGREES)
            topClawPosition -= 1
            bottomClawPosition -= 1
        
    else:
        print("Invalid claw specified. Please choose 'top', 'bottom', or 'both'.")
    
    #so kawaii :3 <3
    brain.screen.clear_line(1)
    brain.screen.set_cursor(1, 1)
    brain.screen.print("so kawaii :3 <3 "+str(topClawPosition) + "  " + str(bottomClawPosition))

while True:
    if controller_1.buttonX.pressing():
        flip_claw("top")
        while controller_1.buttonX.pressing() :
            wait(10, MSEC)

    if controller_1.buttonB.pressing():
        flip_claw("bottom")
        while controller_1.buttonB.pressing() :
            wait(10, MSEC)

    if controller_1.buttonA.pressing():
        flip_claw("both")
        while controller_1.buttonA.pressing() :
            wait(10, MSEC)
    

    #Opens an closes claw
    if controller_1.buttonY.pressing():
        if digital_out_a.value() == False:
            digital_out_a.set(True)
        else:
            digital_out_a.set(False)
        while controller_1.buttonY.pressing() :
            wait(10, MSEC)