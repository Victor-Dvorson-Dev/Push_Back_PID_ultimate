# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Margaret Liu                                                 #
# 	Created:      1/13/2025, 10:24:50 PM                                      #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
import time
from vex import *
# import math

# Parameters Definition and Robots Configuration
brain = Brain()
controller = Controller()
tl = 7.382
tr = 7.382
pi = 3.14159
# #Time_wait=0
wheelFactor = pi * 3.25 / (4/3)
#motor gear teeth 24. wheel gear teeth 36
motorFL = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
motorFR = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False)
motorBL = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)
motorBR = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
motorML = Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
motorMR = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
# Intake_motor_1 = Motor(Ports.PORT13, GearSetting.RATIO_18_1, True)
intakeMotor_1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
# intakeMotor_2 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
# MidRoller = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
TopRoller = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
inertialSensor = Inertial(Ports.PORT3)

arm = DigitalOut(brain.three_wire_port.a)
switch = DigitalOut(brain.three_wire_port.d)
descore = DigitalOut(brain.three_wire_port.f)
controller_1 = Controller(PRIMARY)
# AI Classification Competition Element IDs
class GameElements:
    MOBILE_GOAL = 0
    RED_RING = 1
    BLUE_RING = 2
    SKIP=-1
# AI Vision Color Descriptions
# AI Vision Code Descriptions
AI_clamp = AiVision(Ports.PORT15, AiVision.ALL_AIOBJS)

# hookPneumatic = DigitalOut(brain.three_wire_port.a)
# rightArm = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
# leftArm = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)

def motor_Stop():  
    motorFL.stop()
    motorFR.stop()
    motorML.stop()
    motorMR.stop()
    motorBL.stop()
    motorBR.stop()

def motor_hold():  
    motorFL.stop(HOLD)
    motorFR.stop(HOLD)
    motorML.stop(HOLD)
    motorMR.stop(HOLD)
    motorBL.stop(HOLD)
    motorBR.stop(HOLD)

def motor_brake():  
    motorFL.stop(BRAKE)
    motorFR.stop(BRAKE)
    motorML.stop(BRAKE)
    motorMR.stop(BRAKE)
    motorBL.stop(BRAKE)
    motorBR.stop(BRAKE)

# Commanding the motors based on velocity percentage
def motor_Motion(motorFLSpeed, motorFRSpeed, motorBLSpeed, motorBRSpeed, motorMLSpeed, motorMRSpeed,):
    motorFL.spin(DirectionType.FORWARD, motorFLSpeed, VelocityUnits.PERCENT)
    motorFR.spin(DirectionType.FORWARD, motorFRSpeed, VelocityUnits.PERCENT)
    motorML.spin(DirectionType.FORWARD, motorBLSpeed, VelocityUnits.PERCENT)
    motorMR.spin(DirectionType.FORWARD, motorMRSpeed, VelocityUnits.PERCENT)
    motorBL.spin(DirectionType.FORWARD, motorMLSpeed, VelocityUnits.PERCENT)
    motorBR.spin(DirectionType.FORWARD, motorBRSpeed, VelocityUnits.PERCENT)

# Reset all sensors
def Reset_all():
    motorFL.set_position(0, DEGREES)
    motorFR.set_position(0, DEGREES)
    motorML.set_position(0, DEGREES)
    motorMR.set_position(0, DEGREES)
    motorBL.set_position(0, DEGREES)
    motorBR.set_position(0, DEGREES)
    # inertialSensor.set_rotation(0, DEGREES)

def get_Rotation_Sensor_Position():
    L_current_coor_pre=(motorFL.position(TURNS))
    R_current_coor_pre=(motorBR.position(TURNS))
    return L_current_coor_pre,R_current_coor_pre

def calculate_Rotation_From_Wheels(xLeft, yRight, wheelFactor, tl, tr):
    rotationInRadians = (xLeft * wheelFactor - yRight * wheelFactor) / (tl + tr)
    return rotationInRadians

def autonomousPID(target, initialMaxSpeedLimit, maxSpeedLimit,time_out, export_flag, Kp_l, Kp_r, previousError=[0.0,0.0]):
    # For long distance traveling, we recommand set the initialMaxSpeedLimit to 10 and
    # the maxSpeedLimit to 30. That's OKAY to use higher speed for shorter distance travelling
    # maybe initialMaxSpeedLimit = 25 and maxSpeedLimit = 60. If you start to observe higher errors,
    # try lower these speed setting for better performance.
    # global #Time_wait
    global hook_flag
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")

    Reset_all()
    # PID Constants
    # Defualt K values
    # Default Kp=9
    #working- 5
    Kp = Kp_l   # Proportional constant for linear movement
    Ki = 0.0  # Integral constant for linear movement
    Kd = 0.0   # Derivative constant for linear movement
    #Default KpRotate=44
    #working-22
    KpRotation = Kp_r  # Proportional constant for rotation
    KiRotation = 0.0    # Integral constant for rotation
    KdRotation = 0.0   # Derivative constant for rotation

    currentPosition = [0.0, inertialSensor.rotation()*pi/180] # linear distance and robot heading
    error = [0.0, 0.0]  # movement error and heading error
    integral = 0.0
    headingIntegral = 0.0
    previousError = [0.0, 0.0]
    # initialMaxSpeedLimit = 10
    # maxSpeedLimit = 30
    # Since inertial sensor measures at 50 Hz, so dt will be 0.02
    dt = 0.005  # Time step (20 ms)
    counter = 0
    leftRotation = 0.0
    rightRotation = 0.0

        # PID Loop
    while True:

        # Calculate error
        error[0] = target[0] - currentPosition[0] # distance error
        error[1] = target[1] - currentPosition[1] # heading error
        # error[2] = target[2] - currentPosition[2]
        

        # Calculate integral and derivative
        # Calculate cumulated errors in x and y direction
        integral += error[0] * dt
        # yIntegral += error[1] * dt
        headingIntegral += error[1] * dt

        # Calculate the rate of change of errors in x and y direction
        derivative = (error[0] - previousError[0]) / dt
        # yDerivative = (error[1] - previousError[1]) / dt
        headingDerivative = (error[1] - previousError[1]) / dt

        # Calculate PID output
        xOutput = (Kp * error[0]) + (Ki * integral) + (Kd * derivative)
        # yOutput = (Kp * error[1]) + (Ki * yIntegral) + (Kd * yDerivative)
        turnSpeed = (KpRotation * error[1]) + (KiRotation * headingIntegral) + (KdRotation * headingDerivative)

        # Apply motor speeds for each motor
        motorFLSpeed = xOutput + turnSpeed
        motorFRSpeed = xOutput - turnSpeed
        motorMLSpeed = xOutput + turnSpeed
        motorMRSpeed = xOutput - turnSpeed
        motorBLSpeed = xOutput + turnSpeed
        motorBRSpeed = xOutput - turnSpeed


        # Normalize motor speeds if necessary
        # For the first 100 cycles, limit motor speed to "initialMaxSpeed"
        # And limit motor speed to "maxSpeed" after first 100 cycles
        if counter <= 100:
            maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),abs(motorMLSpeed), abs(motorMRSpeed),
                        abs(motorBLSpeed), abs(motorBRSpeed), initialMaxSpeedLimit)
            if maxSpeed > initialMaxSpeedLimit:
                motorFLSpeed = (motorFLSpeed / maxSpeed) * initialMaxSpeedLimit
                motorFRSpeed = (motorFRSpeed / maxSpeed) * initialMaxSpeedLimit
                motorMLSpeed = (motorMLSpeed / maxSpeed) * initialMaxSpeedLimit
                motorMRSpeed = (motorMRSpeed / maxSpeed) * initialMaxSpeedLimit
                motorBLSpeed = (motorBLSpeed / maxSpeed) * initialMaxSpeedLimit
                motorBRSpeed = (motorBRSpeed / maxSpeed) * initialMaxSpeedLimit
                # motorFLSpeed = ((maxSpeedLimit-initialMaxSpeedLimit)/100)*counter + initialMaxSpeedLimit
                # motorFRSpeed = motorFLSpeed
                # motorBLSpeed = motorFLSpeed
                # motorBRSpeed = motorFLSpeed
        else: # Normalize motor speeds and allow motor speeds to be 100% after 10 cycle   
            maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),abs(motorMLSpeed), abs(motorMRSpeed),
                            abs(motorBLSpeed), abs(motorBRSpeed), maxSpeedLimit)
            if maxSpeed > maxSpeedLimit:
                motorFLSpeed = (motorFLSpeed / maxSpeed) * maxSpeedLimit
                motorFRSpeed = (motorFRSpeed / maxSpeed) * maxSpeedLimit
                motorMLSpeed = (motorMLSpeed / maxSpeed) * maxSpeedLimit
                motorMRSpeed = (motorMRSpeed / maxSpeed) * maxSpeedLimit
                motorBLSpeed = (motorBLSpeed / maxSpeed) * maxSpeedLimit
                motorBRSpeed = (motorBRSpeed / maxSpeed) * maxSpeedLimit

        motor_Motion(motorFLSpeed, motorFRSpeed, motorBLSpeed, motorBRSpeed, motorMLSpeed, motorMRSpeed)

        #shakes intake to prevent stuck rings
        # intake_shake()

        # Update previous errors
        previousError[0] = error[0]
        previousError[1] = error[1]
        # previousError[2] = error[2]

        # Update current position and rotation
        leftRotation, rightRotation = get_Rotation_Sensor_Position()

        # Getting rotation measurements both from wheel rotation and inertial sensor
        # There is a possibility that we might consider using rotation meansurements from wheels for some special scenarios.
        robotHeadingFromWheels = calculate_Rotation_From_Wheels(leftRotation, rightRotation, wheelFactor, tl, tr)
        robotHeadingFromInertial= inertialSensor.rotation()*pi/180
        
        if leftRotation==rightRotation:
            currentPosition[0] = leftRotation*wheelFactor
        else:
            currentPosition[0] = ((leftRotation+rightRotation))/2*wheelFactor
        
        currentPosition[1] = robotHeadingFromInertial
        # currentPosition[1] = robotHeadingFromWheels

        # print out position and error every 0.5 secs
        # if counter % 100 == 0:
        #     print("errors are ")
        #     print(error)
        #     print("/ncurrent position is ")
        #     print(currentPosition)
        #     print("motor speed: " + str(motorFLSpeed))
        #     print(leftRotation)
        #     print(rightRotation)
        #     if robotHeadingFromInertial == robotHeadingFromWheels:
        #         print("Inertial sensor is match the wheels measurement")
        #     else:
        #         print("Sensor: " + str(robotHeadingFromInertial))
        #         print("Wheels meansurement: " + str(robotHeadingFromWheels))

        if counter % 10== 0 and export_flag==1:
            print(counter,end="\t")
            
            print('{:.5f}'.format(error[0]),end="\t")

            print('{:.5f}'.format(error[1]),end="\t")

            print('{:.5f}'.format(xOutput), end="\t")

            print('{:.5f}'.format(turnSpeed), end="\t")

            print('{:.5f}'.format(motorFLSpeed), end="\n")

        # Check if the motor has reached the target or time-out after 6 secs
        # The print-out for trouble shooting. It can be comment out if it's not necessary.

        #working values. 0.01 and 0.015 respectively
        if abs(error[0]) < 0.01 and abs(error[1]) < 0.015:
            print("COMPLETE!!!")
            motor_Stop()
            # time.sleep(0.5)
            print(error)
            print(counter)
            return error
            break
        elif counter > time_out: # A time out after 5 secs (1000*dt)
            print("COMPLETE!!! Timed Out!!!")
            motor_Stop()
            # time.sleep(0.5)
            print(error)
            print(counter)
            return error
            break

        # Wait for the next loop
        time.sleep(dt)
        counter += 1




def set_and_run_rollers(v_low, v_top):
    intakeMotor_1.set_velocity(v_low,PERCENT)
    TopRoller.set_velocity(v_top,PERCENT)
    intakeMotor_1.spin(FORWARD)
    TopRoller.spin(FORWARD)

def all_intake_stop():
    set_and_run_rollers(0,0)

def intake_score(velocity):
    switch.set(False)
    set_and_run_rollers(1*velocity, -1*velocity)

def no_get_stuck(velocity,wait_msec, doWait):
    #wait time is in miliseconds
    time_1=brain.timer.time(MSEC) 
    intake_score(velocity)
    if (doWait == True):
        wait(0.3,SECONDS)
    while (brain.timer.time(MSEC)-time_1)<wait_msec:
        #print(brain.timer.time(MSEC))
        if abs(intakeMotor_1.velocity(PERCENT))<abs(0.2*velocity):
            intakeMotor_1.spin_for(REVERSE,0.2,SECONDS)

        intake_score(velocity)
        wait(0.2,SECONDS)
        print(intakeMotor_1.velocity(PERCENT))
    

        
def intake_hold(velocity):
    switch.set(False)
    set_and_run_rollers(1*velocity, 0.3*velocity)    

def outake(velocity):
    set_and_run_rollers(-1*velocity,0*velocity)

# def user_control():
#     brain.screen.clear_screen()
#     brain.screen.print("driver control")
#     # place driver control in this while loop
#     while True:
#         wait(20, MSEC)
#         forward_speed = controller.axis3.position()  # Forward/Backward (vertical axis)
#         turn_speed = controller.axis4.position()  # Turning (horizontal axis)

#         # Calculate motor speeds
#         left_speed = forward_speed + turn_speed
#         right_speed = forward_speed - turn_speed

#         # Apply joystick values to motors
#         motorFL.spin(DirectionType.FORWARD, left_speed, VelocityUnits.PERCENT)
#         motorFR.spin(DirectionType.FORWARD, right_speed, VelocityUnits.PERCENT)
#         motorBL.spin(DirectionType.FORWARD, left_speed, VelocityUnits.PERCENT)
#         motorBR.spin(DirectionType.FORWARD, right_speed, VelocityUnits.PERCENT)

#         if controller.buttonA.pressing():
#             intakeMotor.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
#         else:
#             intakeMotor.stop()
            
        

# create competition instance
# comp = Competition(user_control, autonomous)
vexcode_brain_precision = 0
vexcode_console_precision = 0
myVariable = 0

"""
CHANGELOG
- first time testing on vincent's field

TO DO:
- retest PID constants (1 for normal speed, 1 for fast?)
- shorten time under 30 secs (not counting actual loader time)
"""


# create a function for handling the starting and stopping of all autonomous tasks
def preauto():

    t_1=brain.timer.time(MSEC)
    inertialSensor.calibrate()
    # Wait until calibration is complete
    while inertialSensor.is_calibrating():
        wait(100, TimeUnits.MSEC)  # Check every 100 ms
    t_2=brain.timer.time(MSEC)

    if t_2-t_1<1800:
        inertialSensor.calibrate()
        while inertialSensor.is_calibrating():
            wait(100, TimeUnits.MSEC)  # Check every 100 ms

    # temp feedback
    print("motorFL: " + str(motorFL.temperature()))
    print("motorFR: " + str(motorFR.temperature()))
    print("motorML: " + str(motorML.temperature()))
    print("motorMR: " + str(motorMR.temperature()))
    print("motorBL: " + str(motorBL.temperature()))
    print("motorBR: " + str(motorBR.temperature()))

    print("\nintake: " + str(intakeMotor_1.temperature()))
    print("top roller: " + str(TopRoller.temperature()))



def vexcode_auton_function():

    extra_buffer=0
    intake_v=90
    print("calibration finished")
    
    export_flag=0
    switch.set(False)
    descore.set(True)
    
    r_offset=math.radians(-180)
    loader_wait=2.5/0.005
    high_goal_wait=3/0.001

    """
    ground floor red side: 0
    ground floor blue side: 4
    upper floor right field blue side: 0
    upper floor left field blue side linear offset: -2
    """

    linear_offset=-.5

    loader1_offset=-1
    loader2_offset=-0.5
    loader3_offset=-0.0
    loader4_offset=-0.5

    # go to the loader
    Time_wait=210
    arm.set(True)
    #previous value:32.25
    f=-32.25-linear_offset
    r=0
    v_min=40
    v_max=75
    Kp_linear=4
    Kp_rotation=30
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    #turn to face first loader
    Time_wait=150
    f=0
    v_min=30
    v_max=50
    r=86
    Kp_linear=4
    Kp_rotation=30
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    #start outake from loader
    intake_hold(100)

    #go into loader
    Time_wait=180
    v_min=30
    v_max=50
    f=-14.6-loader1_offset
    r=90
    Kp_linear=4
    Kp_rotation=30
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+loader_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    #wait(loader_wait,SECONDS)
    """all_intake_stop()"""
    
    #back out of loader
    Time_wait=160
    v_min=50
    v_max=70
    f=13
    r=90
    Kp_linear=4
    Kp_rotation=30
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    arm.set(False)
    #turn to face wall
    Time_wait=150
    v_min=40
    v_max=70
    f=0
    r=0
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)



    #go to allign w/ wall
    Time_wait=150
    v_min=60
    v_max=80       
    """f=-19.5"""
    f=-19.25
    r=0
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    all_intake_stop()
    
    #turn to face second loader
    Time_wait=160
    v_min=50
    v_max=70
    f=0
    r=-85
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    #go to other side
    Time_wait=380
    v_min=65
    v_max=90
    f=-85
    r=-84
    Kp_linear=4
    Kp_rotation=40
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    #rotate to start going to high goal
    Time_wait=150
    v_min=40
    v_max=60
    f=0
    r=0
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    #allign with high goal horizontally
    Time_wait=170
    v_min=40
    v_max=60
    f=12 #previously 12 then 11.5
    r=0
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    

    print("preparing to score 1st time")
    #turn to face high goal    
    Time_wait=170
    v_min=40
    v_max=60
    f=0
    r=-90
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    #go into high goal(right)    
    Time_wait=150
    v_min=40
    v_max=60
    """f=11"""
    f = 12
    r=-90
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    #score blocks from first loader on high goal
    # should be 2.2 seconds
    no_get_stuck(100, high_goal_wait, True)
    #wait(high_goal_wait,SECONDS)
    intake_hold(100)
    arm.set(True)

    #go to second loader
    Time_wait=230
    v_min=30
    v_max=40
    """f=-32"""
    f=-29.5-loader2_offset
    r=-90
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+loader_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    #wait(loader_wait,SECONDS)
    """all_intake_stop()""" # want to continue intaking in case of stray block not getting intaked in time

    #go out of second loader to the high goal
    Time_wait=190
    v_min=40
    """v_max=70"""
    v_max=50
    f=31
    """r=-85.5"""
    r=-90
    Kp_linear=4
    Kp_rotation=30
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    no_get_stuck(100, high_goal_wait, True)
    intake_hold(100)
    #wait(high_goal_wait,SECONDS)

    #go out of high goal
    Time_wait=150
    v_min=40
    v_max=70
    f=-7
    r=-90
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    
    #turn to face third loader
    Time_wait=170
    v_min=40
    v_max=70
    f=0
    r=0
    Kp_linear=4
    Kp_rotation=30
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)

    #go to third loader
    Time_wait=445
    v_min=45
    v_max=80
    """f=96"""
    f=97
    """f=98"""
    r=0
    Kp_linear=4
    Kp_rotation=30
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)

    #turn to face third loader
    Time_wait=180
    v_min=40
    v_max=70
    f=0
    r=-90
    Kp_linear=4
    Kp_rotation=30
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)

    #go into third loader
    Time_wait=220
    v_min=30
    v_max=40
    f=-20-loader3_offset
    r=-90
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+loader_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    #back out of third loader
    Time_wait=170
    v_min=50
    v_max=70
    f=13
    r=-90
    Kp_linear=4
    Kp_rotation=30
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    arm.set(False)
    #turn to face wall
    Time_wait=150
    v_min=40
    v_max=70
    f=0
    r=-180
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    #go to allign w/ wall
    Time_wait=170
    v_min=60
    v_max=70
    f=-24
    r=-180
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    #turn to face fourth loader
    Time_wait=160
    v_min=40
    v_max=70
    f=0
    r=-265
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    #go to original side
    Time_wait=360
    v_min=65
    v_max=90
    f=-80
    r=-265
    Kp_linear=4
    Kp_rotation=30
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    #turn to 4th loader zone
    Time_wait=150
    v_min=40
    v_max=70
    f=0
    r=-360
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    #go to allign with fourth goal horizontally 
    Time_wait=180
    v_min=40
    v_max=50
    f=-14.75
    r=-360
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    #turn to face fourth goal directly
    Time_wait=200
    v_min=30
    v_max=50
    f=0
    r=-270
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    #go in fourth goal
    Time_wait=180
    v_min=50
    v_max=50
    f=20
    r=-267
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    no_get_stuck(100,high_goal_wait, True)
    arm.set(True)

    #new section as of 2/17/26: fourth loader
    intake_hold(100)
    Time_wait=180
    v_min=50
    v_max=50
    f=-30-loader4_offset
    r=-267
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+loader_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    #outtake blocks into the goal
    Time_wait=160
    v_min=50
    v_max=50
    f=31
    r=-267
    Kp_linear=4
    Kp_rotation=25
    """pauses around here..."""
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    no_get_stuck(100,high_goal_wait, False)
    #lift arm up for parking
    arm.set(False)

    # parking time!! back out of goal
    Time_wait=180
    v_min=50
    v_max=70
    f=-16
    r=-272
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    # rotate in preparation for aligning with wall
    Time_wait=170
    v_min=50
    v_max=50
    f=0
    r=-300
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    # align with wall
    Time_wait=280
    v_min=30
    v_max=50
    f=-38
    r=-300
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    
    # face park zone
    Time_wait=180
    v_min=50
    v_max=50
    f=-0
    r=-350
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    arm.set(True)

    # actually park
    Time_wait=250
    v_min=50
    v_max=70
    f=-50 #originally 44
    r=-353
    Kp_linear=4
    Kp_rotation=25
    autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
                  Kp_linear,Kp_rotation)
    arm.set(False)
    #
    return
    
    
def vexcode_driver_function():
    # Start the driver control tasks

    # wait for the driver control period to end
    while( competition.is_driver_control() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the driver control tasks
competition = Competition(vexcode_driver_function, vexcode_auton_function )
preauto()
