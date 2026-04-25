# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       secho                                                        #
# 	Created:      11/24/2025, 8:36:50 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
import time
from vex import *
import math

# Parameters Definition and Robots Configuration
brain = Brain()
controller = Controller()
tl = 7.382
tr = 7.382
pi = 3.14159
# #Time_wait=0
wheelFactor = pi * 3.25 / (3/2)
INCHES_PER_TURN = 7.65
DEGREES_PER_TURN = 67.433

#motor gear teeth 24. wheel gear teeth 36
motorFL = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
motorFR = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False)
motorBL = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)
motorBR = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
motorML = Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
motorMR = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
# Intake_motor_1 = Motor(Ports.PORT13, GearSetting.RATIO_18_1, True)
intakeMotor_1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False) # Originally True
intakeMotor_2 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
# MidRoller = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
# TopRoller = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)
inertialSensor = Inertial(Ports.PORT3)
arm = DigitalOut(brain.three_wire_port.a)
descore = DigitalOut(brain.three_wire_port.f)
switch = DigitalOut(brain.three_wire_port.h)
controller_1 = Controller(PRIMARY)

# MOVEMENT FUNCTIONS
#set velocity of all drivetrain motors
def set_velocity_all(velocity):
    set_velocity_left(velocity)
    set_velocity_right(velocity)

#set velocity of all left motors
def set_velocity_left(velocity):
    motorFL.set_velocity(velocity, PERCENT)
    motorML.set_velocity(velocity, PERCENT)
    motorBL.set_velocity(velocity, PERCENT)

#set velocity of all right motors
def set_velocity_right(velocity):
    motorFR.set_velocity(velocity, PERCENT)
    motorMR.set_velocity(velocity, PERCENT)
    motorBR.set_velocity(velocity, PERCENT)

#drive forward [inches] inches
def drive_Forward(inches):
    dist = inches/INCHES_PER_TURN
    motorFL.spin_for(FORWARD, dist, TURNS, wait=False)
    motorFR.spin_for(FORWARD, dist, TURNS, wait=False)
    motorML.spin_for(FORWARD, dist, TURNS, wait=False)
    motorMR.spin_for(FORWARD, dist, TURNS, wait=False)
    motorBL.spin_for(FORWARD, dist, TURNS, wait=False)
    motorBR.spin_for(FORWARD, dist, TURNS)

#drive backward [inches] inches
def drive_Reverse(inches):
    dist = inches/INCHES_PER_TURN
    motorFL.spin_for(REVERSE, dist, TURNS, wait=False)
    motorFR.spin_for(REVERSE, dist, TURNS, wait=False)
    motorML.spin_for(REVERSE, dist, TURNS, wait=False)
    motorMR.spin_for(REVERSE, dist, TURNS, wait=False)
    motorBL.spin_for(REVERSE, dist, TURNS, wait=False)
    motorBR.spin_for(REVERSE, dist, TURNS)

def rotate_CW(degrees):
    turn = degrees/DEGREES_PER_TURN
    motorFL.spin_for(FORWARD, turn, TURNS, wait=False)
    motorFR.spin_for(REVERSE, turn, TURNS, wait=False)
    motorML.spin_for(FORWARD, turn, TURNS, wait=False)
    motorMR.spin_for(REVERSE, turn, TURNS, wait=False)
    motorBL.spin_for(FORWARD, turn, TURNS, wait=False)
    motorBR.spin_for(REVERSE, turn, TURNS)

def rotate_CCW(degrees):
    turn = degrees/DEGREES_PER_TURN
    motorFL.spin_for(REVERSE, turn, TURNS, wait=False)
    motorFR.spin_for(FORWARD, turn, TURNS, wait=False)
    motorML.spin_for(REVERSE, turn, TURNS, wait=False)
    motorMR.spin_for(FORWARD, turn, TURNS, wait=False)
    motorBL.spin_for(REVERSE, turn, TURNS, wait=False)
    motorBR.spin_for(FORWARD, turn, TURNS)

# stop the drivetrain motors (!?)
def motor_Stop():  
    motorFL.stop()
    motorFR.stop()
    motorML.stop()
    motorMR.stop()
    motorBL.stop()
    motorBR.stop()

# Commanding the motors based on velocity percentage
def motor_Motion(motorFLSpeed, motorFRSpeed, motorBLSpeed, motorBRSpeed, motorMLSpeed, motorMRSpeed,):
    motorFL.spin(DirectionType.FORWARD, motorFLSpeed, VelocityUnits.PERCENT)
    motorFR.spin(DirectionType.FORWARD, motorFRSpeed, VelocityUnits.PERCENT)
    motorML.spin(DirectionType.FORWARD, motorMLSpeed, VelocityUnits.PERCENT)
    motorMR.spin(DirectionType.FORWARD, motorMRSpeed, VelocityUnits.PERCENT)
    motorBL.spin(DirectionType.FORWARD, motorBLSpeed, VelocityUnits.PERCENT)
    motorBR.spin(DirectionType.FORWARD, motorBRSpeed, VelocityUnits.PERCENT)

# Reset all drivetrain motors
def reset_Drive():
    motorFL.set_position(0, DEGREES)
    motorFR.set_position(0, DEGREES)
    motorML.set_position(0, DEGREES)
    motorMR.set_position(0, DEGREES)
    motorBL.set_position(0, DEGREES)
    motorBR.set_position(0, DEGREES)
    # inertialSensor.set_rotation(0, DEGREES)

# SCORING MECH FUNCTIONS
# velocity of intakes
def set_velocity_intake(velocity):
    intakeMotor_1.set_velocity(velocity, PERCENT)
    intakeMotor_2.set_velocity(velocity, PERCENT)
    # MidRoller.set_velocity(velocity, PERCENT)
    # TopRoller.set_velocity(velocity, PERCENT)

# sentences a block to the basket
def intake():
    intakeMotor_1.spin(REVERSE)
    intakeMotor_2.stop()
    # MidRoller.spin(REVERSE)
    # TopRoller.stop()

# ejects a block through the intake
def outtake():
    intakeMotor_1.spin(FORWARD)
    intakeMotor_2.stop()
    # MidRoller.spin(FORWARD)
    # TopRoller.stop()

# basket to top roller
def hightake():
    intakeMotor_1.spin(FORWARD)
    intakeMotor_2.spin(FORWARD)
    # MidRoller.spin(REVERSE)
    # TopRoller.spin(REVERSE)

# stops all intake motors
def stop_Intake():
    intakeMotor_1.stop()
    intakeMotor_2.stop()
    # MidRoller.stop()
    # TopRoller.stop()

# TRACKING WHEEL FUNCTIONS
def get_Rotation_Sensor_Position():
    L_current_coor_pre=(motorFL.position(TURNS))
    R_current_coor_pre=(motorBR.position(TURNS))
    return L_current_coor_pre,R_current_coor_pre

def calculate_Rotation_From_Wheels(xLeft, yRight, wheelFactor, tl, tr):
    rotationInRadians = (xLeft * wheelFactor - yRight * wheelFactor) / (tl + tr)
    return rotationInRadians

# THE BIG MAN HIMSELF
def autonomousPID(target, initialMaxSpeedLimit, maxSpeedLimit,time_out, export_flag, previousError=[0.0,0.0], auto_deadline_sec=0.0):
    # For long distance traveling, we recommand set the initialMaxSpeedLimit to 10 and
    # the maxSpeedLimit to 30. That's OKAY to use higher speed for shorter distance travelling
    # maybe initialMaxSpeedLimit = 25 and maxSpeedLimit = 60. If you start to observe higher errors,
    # try lower these speed setting for better performance.
    global Time_wait
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")

    reset_Drive()
    # PID Constants
    # Defualt K values
    # Default Kp=9
    Kp = 5.0   # Proportional constant for linear movement
    Ki = 0.0  # Integral constant for linear movement
    Kd = 0.0   # Derivative constant for linear movement
    #Default KpRotate=44
    KpRotation = 30.0    # Proportional constant for rotation
    KiRotation = 0.0    # Integral constant for rotation
    KdRotation = 0.5   # Derivative constant for rotation

    currentPosition = [0.0, 0.0] # linear distance and robot heading
    error = [0.0, 0.0]  # movement error and heading error
    integral = 0.0
    headingIntegral = 0.0
    # Use the passed-in previousError (don't overwrite it)
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
        # print("Motor speeds: ",motorFLSpeed,motorFRSpeed,motorBLSpeed,motorBR,motorMLSpeed,motorMRSpeed)
        #shakes intake to prevent stuck rings
        # intake_shake()

        # Update previous errors
        previousError[0] = error[0]
        previousError[1] = error[1]
        # previousError[2] = error[2]

        # Update current position and rotation
        leftRotation, rightRotation = get_Rotation_Sensor_Position()
        # print("Left Rotation",leftRotation)
        # print("Right Rotation",rightRotation)

        # Getting rotation measurements both from wheel rotation and inertial sensor
        # There is a possibility that we might consider using rotation meansurements from wheels for some special scenarios.
        robotHeadingFromWheels = calculate_Rotation_From_Wheels(leftRotation, rightRotation, wheelFactor, tl, tr)
        robotHeadingFromInertial= inertialSensor.rotation()*pi/180
        
        if leftRotation==rightRotation:
            currentPosition[0] = leftRotation*wheelFactor
        else:
            currentPosition[0] = ((leftRotation+rightRotation))/2*wheelFactor
        
        currentPosition[1] = robotHeadingFromInertial
        # print("Changed",currentPosition[0],currentPosition[1])
        # currentPosition[1] = robotHeadingFromWheels

        # Compact debug: one-line status every 50 iterations
        if counter % 50 == 0:
            elapsed = counter * dt
            print("n={:d} {:.2f}s pos={:.2f}in/{:.1f}deg err={:.2f}in/{:.1f}deg spd={:.1f}% turn={:.1f}%".format(
                counter, elapsed,
                currentPosition[0], math.degrees(currentPosition[1]),
                error[0], math.degrees(error[1]),
                xOutput, turnSpeed))

        # Tab-separated export every 10 iterations (for spreadsheet analysis)
        if counter % 10 == 0 and export_flag == 1:
            print("{:d}\t{:.3f}\t{:.3f}\t{:.3f}\t{:.3f}\t{:.3f}".format(
                counter, error[0], error[1], xOutput, turnSpeed, motorFLSpeed))

        # Check if the motor has reached the target or time-out after 6 secs
        # The print-out for trouble shooting. It can be comment out if it's not necessary.
        # 0.01 and 0.015
        if abs(error[0]) < 0.3 and abs(error[1]) < 0.05:
            motor_Stop()
            elapsed = counter * dt
            print("OK t={:.2f}in/{:.1f}deg err={:.2f}in/{:.1f}deg n={:d} {:.2f}s".format(
                target[0], math.degrees(target[1]),
                error[0], math.degrees(error[1]),
                counter, elapsed))
            return error
        elif counter > time_out:
            motor_Stop()
            elapsed = counter * dt
            print("TIMEOUT t={:.2f}in/{:.1f}deg err={:.2f}in/{:.1f}deg n={:d} {:.2f}s".format(
                target[0], math.degrees(target[1]),
                error[0], math.degrees(error[1]),
                counter, elapsed))
            return error
        elif auto_deadline_sec > 0 and brain.timer.time(SECONDS) >= auto_deadline_sec:
            motor_Stop()
            elapsed = counter * dt
            print("TIME_UP t={:.2f}in/{:.1f}deg err={:.2f}in/{:.1f}deg n={:d} {:.2f}s brain={:.1f}s".format(
                target[0], math.degrees(target[1]),
                error[0], math.degrees(error[1]),
                counter, elapsed, brain.timer.time(SECONDS)))
            return error

        # Wait for the next loop
        time.sleep(dt)
        counter += 1
def run_intake(v1,v2):
    intakeMotor_1.set_velocity(v1,PERCENT)
    intakeMotor_2.set_velocity(v2,PERCENT)
    intakeMotor_1.spin(FORWARD)
    intakeMotor_2.spin(FORWARD)
def all_intake_stop():
    run_intake(0,0)
def score_long(velocity):
    switch.set(False)
    run_intake(1*velocity,1*velocity)
def hold_long(velocity):
    switch.set(False)
    run_intake(1*velocity,-0.3*velocity)
def score_mid(velocity):
    switch.set(True)
    run_intake(1*velocity,1*velocity)
def score_low(velocity):
    run_intake(-1*velocity,-1*velocity)
# def set_and_run_rollers(v_low,v_mid,v_top):
#     intakeMotor_1.set_velocity(v_low,PERCENT)
#     intakeMotor_2.set_velocity(v_low,PERCENT)
#     # MidRoller.set_velocity(v_mid,PERCENT)
#     # TopRoller.set_velocity(v_top,PERCENT)
#     intakeMotor_1.spin(FORWARD)
#     intakeMotor_2.spin(FORWARD)
#     # MidRoller.spin(FORWARD)
#     # TopRoller.spin(FORWARD)

# def all_intake_stop():
#     set_and_run_rollers(0,0,0)

# def intake_ground_to_mid(velocity):
#     set_and_run_rollers(-1*velocity,-1*velocity,1*velocity)

# def intake_ground_to_top(velocity):
#     set_and_run_rollers(-1*velocity,-1*velocity,-1*velocity)

# def intake_ground_to_basket(velocity):
#     set_and_run_rollers(-1*velocity,1*velocity,0*velocity)    

# def intake_ground_hold_at_top(velocity):
#     #topv orgininaly -0.6
#     set_and_run_rollers(-0.8*velocity,-1*velocity,-0.8*velocity)

# def outake_top_to_ground(velocity):
#     set_and_run_rollers(1*velocity,-0.5*velocity,-1*velocity)

# def outake_basket_to_ground(velocity):
#     set_and_run_rollers(-1*velocity,-1*velocity,0*velocity)

# def outake_basket_to_top(velocity):
#     set_and_run_rollers(-1*velocity, -1*velocity, -1*velocity)
def motorSetHold():
    motorFL.set_stopping(HOLD)
    motorFR.set_stopping(HOLD)
    motorML.set_stopping(HOLD)
    motorMR.set_stopping(HOLD)
    motorBL.set_stopping(HOLD)
    motorBR.set_stopping(HOLD)

def motorSetCoaset():
    motorFL.set_stopping(COAST)
    motorFR.set_stopping(COAST)
    motorML.set_stopping(COAST)
    motorMR.set_stopping(COAST)
    motorBL.set_stopping(COAST)
    motorBR.set_stopping(COAST)

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
            
        

vexcode_brain_precision = 0
vexcode_console_precision = 0
myVariable = 0




# create a function for handling the starting and stopping of all autonomous tasks
def vexcode_auton_function():
    # Start the autonomous control tasks
    brain.timer.reset()
    brain.screen.clear_screen()
    brain.screen.print("auto time")
    print("auto has started")
    
    DEADLINE = 14.9
    biasDegrees = 0
    v_min = 40
    v_max = 50

    # goes to the three balls in the center
    print("Brain time:",str(brain.timer.time(SECONDS)))
    descore.set(False)
    hold_long(100)
    # autonomousPID([0, math.radians(biasDegrees)], v_min, v_max, 100, export_flag)
    autonomousPID([-24.5, math.radians(0+biasDegrees)], v_min, v_max, 200, export_flag, auto_deadline_sec=DEADLINE)

    # goes to lower goal
    autonomousPID([0, math.radians(-80+biasDegrees)], v_min, v_max, 150, export_flag, auto_deadline_sec=DEADLINE)
    autonomousPID([-14, math.radians(-80+biasDegrees)], v_min, v_max, 170, export_flag, auto_deadline_sec=DEADLINE)
    score_low(50)
    wait(0.9,SECONDS)
    v_min = 60
    v_max = 70
    autonomousPID([11, math.radians(-80+biasDegrees)], v_min, v_max, 100, export_flag, auto_deadline_sec=DEADLINE)
    stop_Intake()
    # goes to the loader
    autonomousPID([0, math.radians(101+biasDegrees)], v_min, v_max, 220, export_flag, auto_deadline_sec=DEADLINE)
    arm.set(True)
    autonomousPID([-36.5, math.radians(101+biasDegrees)], v_min, v_max, 250, export_flag, auto_deadline_sec=DEADLINE)
    hold_long(100)
    wait(0.25,SECONDS)
    v_min = 30
    v_max = 40
    autonomousPID([0, math.radians(145+biasDegrees)], v_min, v_max, 200, export_flag, auto_deadline_sec=DEADLINE)
    autonomousPID([-8.5, math.radians(145+biasDegrees)], v_min, v_max, 200, export_flag, auto_deadline_sec=DEADLINE)
    # scores in the high goal
    v_min = 40
    v_max = 50
    autonomousPID([30, math.radians(145+biasDegrees)], v_min, v_max, 200, export_flag, auto_deadline_sec=DEADLINE)
    #autonomousPID([15, math.radians(148)], v_min, v_max, 235, export_flag)

    score_long(100)
    wait(1.5,SECONDS)
    stop_Intake()
    autonomousPID([-6, math.radians(145+biasDegrees)], v_min, v_max, 100, export_flag, auto_deadline_sec=DEADLINE)
    # control zone
    arm.set(False)
    autonomousPID([0, math.radians(237+biasDegrees)], v_min, v_max, 200, export_flag, auto_deadline_sec=DEADLINE)
    descore.set(True)
    autonomousPID([-10, math.radians(237+biasDegrees)], v_min, v_max, 120, export_flag, auto_deadline_sec=DEADLINE)
    autonomousPID([0, math.radians(139+biasDegrees)], v_min, v_max, 200, export_flag, auto_deadline_sec=DEADLINE)
    descore.set(False)
    v_min = 60
    v_max = 70
    # print("Brain time:",str(brain.timer.time(SECONDS)))
    autonomousPID([21, math.radians(149+biasDegrees)], v_min, v_max, 200, export_flag, auto_deadline_sec=DEADLINE)

    #autonomousPID([12, math.radians(235)], v_min, v_max, 200, export_flag)
    print("Brain time",str(brain.timer.time(SECONDS)))
    descore.set(True)
    motor_Stop()


"""#put in those blocks
hightake()
wait(3, SECONDS)
all_intake_stop()"""


"""#backtrack
drive_Reverse(49)
rotate_CW(70)
drive_Reverse(30)
rotate_CW(180)"""



def onauton_autonomous_0():
    global myVariable
    pass
def when_started1():
    print("motorFL: " + str(motorFL.temperature()))
    print("motorFR: " + str(motorFR.temperature()))
    print("motorML: " + str(motorML.temperature()))
    print("motorMR: " + str(motorMR.temperature()))
    print("motorBL: " + str(motorBL.temperature()))
    print("motorBR: " + str(motorBR.temperature()))

    print("\nintake 1: " + str(intakeMotor_1.temperature()))
    print("intake 2: " + str(intakeMotor_2.temperature()))
    # print("top roller: " + str(TopRoller.temperature()))
    # print("mid roller: " + str(MidRoller.temperature()))

    print("preauto")

    t_1=brain.timer.time(MSEC)
    inertialSensor.calibrate()

    set_velocity_all(30)
    set_velocity_intake(100)
        
    global Time_wait
    global extra_buffer
    global export_flag
    
    # Wait until calibration is complete
    while inertialSensor.is_calibrating():
        wait(100, TimeUnits.MSEC)  # Check every 100 ms
    t_2=brain.timer.time(MSEC)

    if t_2-t_1<1800:
        inertialSensor.calibrate()
        while inertialSensor.is_calibrating():
            wait(100, TimeUnits.MSEC)  # Check every 100 ms


    default_time=200

    
    Time_wait=default_time
    extra_buffer = -11
    intake_v=90
    print("calibration finished")
    v_min=12
    v_max=60
    export_flag=0
    r_offset=math.radians(0)
    Time_wait=400
    r=0

    

    # outake_top_to_ground(100)
    # return
    # f=10
    # r=0
    # autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag)
    # return
    #turn to face cluster of 3 blocks

    #move towards the blocks"

    #f is the foward value (local)
    #r is the rotation value (global)
    #Time_wait is the amount of time it gets to run
    #most scoring functions have been made. (see lines 263-293)
    #Contact me (Margaret Liu) on discord if you have questions.
    #f=0
    #r=0
    #autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag)

    
def vexcode_driver_function():
    # Start the driver control tasks

    # wait for the driver control period to end
    while True:
        # wait 10 milliseconds before checking again
        wait( 20, MSEC )
    # Stop the driver control tasks

# create competition instance
comp = Competition(vexcode_driver_function, vexcode_auton_function)
when_started1()


