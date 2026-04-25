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
import math
from vex import *

# Parameters Definition and Robots Configuration
brain = Brain()
controller = Controller()
tl = 7.382
tr = 7.382
pi = 3.14159
# #Time_wait=0
wheelFactor = pi * 3.25 / (3/2)

#motor gear teeth 24. wheel gear teeth 36
# Motor Configuration
motorFL = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
motorFR = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False)
motorBL = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)
motorBR = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
motorML = Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
motorMR = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)

# Intake and Roller Motors
intakeMotor_1 = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)
TopRoller = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)

# Sensors and Pneumatics
inertialSensor = Inertial(Ports.PORT3)
arm = DigitalOut(brain.three_wire_port.a)
cap = DigitalOut(brain.three_wire_port.h)
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
frontDist = Distance(Ports.PORT9)
rightDist = Distance(Ports.PORT10)

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

def autonomousPID(
    target,
    initialMaxSpeedLimit,
    maxSpeedLimit,
    time_out,
    export_flag,
    Kp_l,
    Kp_r,
    previousError=None,
    exit_velocity_pct=0.0,
    slow_down_distance=6.0,
    settle_error_dist=0.5,
    settle_error_heading=0.03,
    settle_loops=8,
    stop_at_end=True,
    heading_guard_deg=2.0,
    heading_guard_floor=0.0,
):
    """
    Drive a straight or turning motion using simple PID:
      - Encoders for distance
      - Inertial sensor for heading
    Simplified: no IMU acceleration, no drift PID.

    target: [target_distance_inches, target_heading_radians]
    exit_velocity_pct: keep this forward speed (percent) when finishing so
      the next move can chain without a dead stop. Sign follows target distance.
    slow_down_distance: within this many inches remaining, cap speed toward
      exit_velocity_pct to create a soft decel instead of a hard stop.
    settle_*: settle window thresholds and consecutive loop count required
      before declaring the move complete.
    stop_at_end: if False and exit_velocity_pct != 0, the loop returns while
      motors stay commanded, allowing back-to-back profiles.
    heading_guard_deg: above this heading error, forward output is scaled.
    heading_guard_floor: minimum forward scaling (0 keeps full suppression).
    """

    global hook_flag
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")

    Reset_all()

    if previousError is None:
        previousError = [0.0, 0.0]
    # --- PID constants ---
    Kp = Kp_l      # linear P
    Ki = 0.0

    KpRotation = Kp_r  # heading P
    KiRotation = 0.0

    # Derivative terms (optionally controlled by globals for testing)
    try:
        Kd = Kd_linear_global
    except NameError:
        Kd = 0.0

    try:
        KdRotation = Kd_rotation_global
    except NameError:
        KdRotation = 0.0

    # currentPosition[0] = distance (inches)
    # currentPosition[1] = heading (radians)
    currentPosition = [0.0, inertialSensor.rotation() * pi / 180.0]

    error = [0.0, 0.0]          # [distance_error, heading_error]
    integral = 0.0              # distance integral
    headingIntegral = 0.0       # heading integral

    dt = 0.005                  # 200 Hz loop
    counter = 0
    settle_counter = 0
    leftRotation = 0.0
    rightRotation = 0.0

    # ---------- Heading guard settings ----------
    # When heading error is larger than this angle, scale down forward output.
    # User-tunable heading guard. When heading error exceeds this angle,
    # forward output is scaled down to prioritize turning. Set high (e.g., 45)
    # or 0 to effectively disable. heading_guard_floor keeps some forward feed.
    HEADING_GUARD_DEG = heading_guard_deg
    # -------------------------------------------

    while True:
        # --- 1. Compute errors ---
        error[0] = target[0] - currentPosition[0]   # distance error (inches)
        error[1] = target[1] - currentPosition[1]   # heading error (radians)

        # --- 2. Integrals & derivatives ---
        integral += error[0] * dt
        headingIntegral += error[1] * dt

        derivative = (error[0] - previousError[0]) / dt
        headingDerivative = (error[1] - previousError[1]) / dt

        # --- 3. Raw PID outputs (before guards/limits) ---
        xOutput = (Kp * error[0]) + (Ki * integral) + (Kd * derivative)

        turnSpeed = (KpRotation * error[1] +
                     KiRotation * headingIntegral +
                     KdRotation * headingDerivative)

        # ---------- HEADING GUARD ON FORWARD ----------
        # Convert heading error to degrees for intuitive thresholding
        heading_error_deg = error[1] * 180.0 / pi
        abs_head_err = abs(heading_error_deg)

        if HEADING_GUARD_DEG > 0 and abs_head_err > HEADING_GUARD_DEG:
            # Scale forward down when heading is off.
            scale = HEADING_GUARD_DEG / abs_head_err
            if scale < heading_guard_floor:
                scale = heading_guard_floor
            elif scale > 1.0:
                scale = 1.0
            xOutput *= scale
        # -------------------------------------------------

        # --- 4. Mix into individual motor speeds ---
        motorFLSpeed = xOutput + turnSpeed
        motorFRSpeed = xOutput - turnSpeed
        motorMLSpeed = xOutput + turnSpeed
        motorMRSpeed = xOutput - turnSpeed
        motorBLSpeed = xOutput + turnSpeed
        motorBRSpeed = xOutput - turnSpeed

        # --- 5. Speed limiting with optional soft decel profile ---
        # Dynamic cap near the goal to avoid a hard stop. When far, use the
        # provided maxSpeedLimit; inside slow_down_distance, linearly blend
        # toward exit_velocity_pct.
        distance_remaining = abs(error[0])
        exit_speed = abs(exit_velocity_pct)
        dynamic_limit = maxSpeedLimit
        if slow_down_distance > 0:
            blend = min(1.0, max(0.0, distance_remaining / slow_down_distance))
            dynamic_limit = exit_speed + (maxSpeedLimit - exit_speed) * blend

        if counter <= 100:
            maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),
                           abs(motorMLSpeed), abs(motorMRSpeed),
                           abs(motorBLSpeed), abs(motorBRSpeed),
                           initialMaxSpeedLimit)
            if maxSpeed > initialMaxSpeedLimit:
                motorFLSpeed = (motorFLSpeed / maxSpeed) * initialMaxSpeedLimit
                motorFRSpeed = (motorFRSpeed / maxSpeed) * initialMaxSpeedLimit
                motorMLSpeed = (motorMLSpeed / maxSpeed) * initialMaxSpeedLimit
                motorMRSpeed = (motorMRSpeed / maxSpeed) * initialMaxSpeedLimit
                motorBLSpeed = (motorBLSpeed / maxSpeed) * initialMaxSpeedLimit
                motorBRSpeed = (motorBRSpeed / maxSpeed) * initialMaxSpeedLimit
        else:
            maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),
                           abs(motorMLSpeed), abs(motorMRSpeed),
                           abs(motorBLSpeed), abs(motorBRSpeed),
                           dynamic_limit)
            if maxSpeed > dynamic_limit:
                motorFLSpeed = (motorFLSpeed / maxSpeed) * dynamic_limit
                motorFRSpeed = (motorFRSpeed / maxSpeed) * dynamic_limit
                motorMLSpeed = (motorMLSpeed / maxSpeed) * dynamic_limit
                motorMRSpeed = (motorMRSpeed / maxSpeed) * dynamic_limit
                motorBLSpeed = (motorBLSpeed / maxSpeed) * dynamic_limit
                motorBRSpeed = (motorBRSpeed / maxSpeed) * dynamic_limit

        # --- 6. Command the motors ---
        motor_Motion(motorFLSpeed, motorFRSpeed,
                     motorBLSpeed, motorBRSpeed,
                     motorMLSpeed, motorMRSpeed)

        # --- 7. Save previous errors ---
        previousError[0] = error[0]
        previousError[1] = error[1]

        # --- 8. Update position from encoders & inertial heading ---
        leftRotation, rightRotation = get_Rotation_Sensor_Position()

        robotHeadingFromWheels = calculate_Rotation_From_Wheels(
            leftRotation, rightRotation, wheelFactor, tl, tr
        )
        robotHeadingFromInertial = inertialSensor.rotation() * pi / 180.0

        if leftRotation == rightRotation:
            currentPosition[0] = leftRotation * wheelFactor
        else:
            currentPosition[0] = ((leftRotation + rightRotation)) / 2.0 * wheelFactor

        currentPosition[1] = robotHeadingFromInertial
        # currentPosition[1] = robotHeadingFromWheels  # optional alternative

        # --- 9. Optional debug printout ---
        if counter % 10 == 0 and export_flag == 1:
            print(counter, end="\t")
            print('{:.5f}'.format(error[0]), end="\t")
            print('{:.5f}'.format(error[1]), end="\t")
            print('{:.5f}'.format(xOutput), end="\t")
            print('{:.5f}'.format(turnSpeed), end="\t")
            print('{:.5f}'.format(motorFLSpeed), end="\n")

        # --- 10. Exit conditions with settle window ---
        if abs(error[0]) < settle_error_dist and abs(error[1]) < settle_error_heading:
            settle_counter += 1
        else:
            settle_counter = 0

        if settle_counter >= settle_loops:
            print("COMPLETE!!!")
            if stop_at_end or exit_velocity_pct == 0.0:
                motor_Stop()
            else:
                # Maintain a small exit velocity in the direction of travel
                forward = exit_velocity_pct if target[0] >= 0 else -exit_velocity_pct
                motorFLSpeed = forward + turnSpeed
                motorFRSpeed = forward - turnSpeed
                motorMLSpeed = forward + turnSpeed
                motorMRSpeed = forward - turnSpeed
                motorBLSpeed = forward + turnSpeed
                motorBRSpeed = forward - turnSpeed
                motor_Motion(motorFLSpeed, motorFRSpeed,
                             motorBLSpeed, motorBRSpeed,
                             motorMLSpeed, motorMRSpeed)
            print(error)
            print(counter)
            return error
        elif counter > time_out:
            print("COMPLETE!!! Timed Out!!!")
            motor_Stop()
            print(error)
            print(counter)
            return error

        time.sleep(dt)
        counter += 1


def read_dist_mm_filtered(sensor, samples=3, sample_delay_ms=0, min_mm=20, max_mm=2000, fallback=None):
    values = []
    for _ in range(max(1, samples)):
        mm = sensor.object_distance(MM)
        if min_mm <= mm <= max_mm:
            values.append(mm)
        if sample_delay_ms > 0:
            wait(sample_delay_ms, MSEC)

    if values:
        values.sort()
        mid = len(values) // 2
        if len(values) % 2 == 1:
            return values[mid]
        return (values[mid - 1] + values[mid]) / 2.0

    if fallback is not None:
        return fallback
    return max_mm


def finalize_to_front_wall(
    target_mm,
    timeout_ms=2500,
    kp=0.12,
    ki=0.0,
    kd=0.03,
    max_speed_pct=35.0,
    min_speed_pct=7.0,
    tolerance_mm=12.0,
    settle_loops=8,
    control_period_ms=5,
    sensor_period_ms=20,
    integral_limit=6000.0,
):
    dt = control_period_ms / 1000.0
    integral = 0.0
    settle_count = 0
    start_ms = brain.timer.time(MSEC)
    last_sensor_ms = start_ms - sensor_period_ms
    last_dist = read_dist_mm_filtered(frontDist)
    last_error = last_dist - target_mm

    while (brain.timer.time(MSEC) - start_ms) < timeout_ms:
        loop_start_ms = brain.timer.time(MSEC)
        sensor_updated = False
        if (loop_start_ms - last_sensor_ms) >= sensor_period_ms:
            last_dist = read_dist_mm_filtered(frontDist, fallback=last_dist)
            sensor_updated = True
            last_sensor_ms = loop_start_ms

        dist_mm = last_dist

        error = dist_mm - target_mm
        integral += error * dt
        if integral > integral_limit:
            integral = integral_limit
        elif integral < -integral_limit:
            integral = -integral_limit
        if sensor_updated:
            derivative_dt = sensor_period_ms / 1000.0
            if derivative_dt <= 0:
                derivative_dt = dt
            derivative = (error - last_error) / derivative_dt
            last_error = error
        else:
            derivative = 0.0
        output = kp * error + ki * integral + kd * derivative

        if output > max_speed_pct:
            output = max_speed_pct
        elif output < -max_speed_pct:
            output = -max_speed_pct

        if abs(error) > tolerance_mm and abs(output) < min_speed_pct:
            output = min_speed_pct if output >= 0 else -min_speed_pct

        motor_Motion(output, output, output, output, output, output)

        if abs(error) <= tolerance_mm:
            settle_count += 1
        else:
            settle_count = 0

        if settle_count >= settle_loops:
            break

        loop_elapsed_ms = brain.timer.time(MSEC) - loop_start_ms
        remaining_ms = control_period_ms - loop_elapsed_ms
        if remaining_ms > 0:
            wait(remaining_ms, MSEC)

    motor_Stop()
    return last_dist - target_mm

# def set_and_run_rollers(v_low,v_mid,v_top):
#     intakeMotor_1.set_velocity(v_low,PERCENT)
#     intakeMotor_2.set_velocity(v_low,PERCENT)
#     MidRoller.set_velocity(v_mid,PERCENT)
#     TopRoller.set_velocity(v_top,PERCENT)
#     intakeMotor_1.spin(FORWARD)
#     intakeMotor_2.spin(FORWARD)
#     MidRoller.spin(FORWARD)
#     TopRoller.spin(FORWARD)

# def all_intake_stop():
#     set_and_run_rollers(0,0,0)

# def intake_ground_to_mid(velocity):
#     set_and_run_rollers(-1*velocity,-1*velocity,1*velocity)

# def intake_ground_to_top(velocity):
#     set_and_run_rollers(-0.7*velocity,-1*velocity,-1*velocity)

# def intake_mid_to_top(velocity):
#     set_and_run_rollers(0*velocity,-1*velocity,-1*velocity)

# def intake_ground_to_basket(velocity):
#     set_and_run_rollers(1*velocity,1*velocity,1*velocity)

# def intake_ground_cycle_at_top(velocity):
#     #topv orgininaly -0.6
#     set_and_run_rollers(-0.8*velocity,-1*velocity,-0.8*velocity)

# def intake_ground_hold_in_basket(velocity):
#     #topv orgininaly -0.6
#     set_and_run_rollers(-0.7*velocity,1*velocity,0*velocity)

# def intake_ground_hold_in_basket_top(velocity):
#     #topv orgininaly -0.6
#     set_and_run_rollers(-1*velocity,0*velocity,-0.5*velocity)

# def outake_top_to_ground(velocity):
#     set_and_run_rollers(1*velocity,-0.5*velocity,-1*velocity)

# def outake_basket_to_ground(velocity):
#     set_and_run_rollers(-1*velocity,-1*velocity,0*velocity)



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




# create a function for handling the starting and stopping of all autonomous tasks
def vexcode_auton_function():
    # Start the autonomous control tasks
    auton_task_0 = Thread( onauton_autonomous_0 )
    # wait for the driver control period to end
    while( competition.is_autonomous() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the autonomous control tasks
    auton_task_0.stop()
def when_started1():
    global myVariable
    # Optional: auto-run one of the test profiles when code starts.
    # Set TEST_TO_RUN (above) to "move60", "arc", "chain", "turn", or "smooth".
    if TEST_TO_RUN is not None:
        run_selected_test(TEST_TO_RUN)
def onauton_autonomous_0():

    t_1=brain.timer.time(MSEC)
    inertialSensor.calibrate()

    # global #Time_wait
    # Wait until calibration is complete
    while inertialSensor.is_calibrating():
        wait(100, TimeUnits.MSEC)  # Check every 100 ms
    t_2=brain.timer.time(MSEC)

    if t_2-t_1<1800:
        inertialSensor.calibrate()
        while inertialSensor.is_calibrating():
            wait(100, TimeUnits.MSEC)  # Check every 100 ms



def vexcode_driver_function():
    # Start the driver control tasks

    # wait for the driver control period to end
    while( competition.is_driver_control() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the driver control tasks
competition = Competition(vexcode_driver_function, vexcode_auton_function )
when_started1()

# ------------------------------------------------------------
# PID tuning helper functions
# Each test moves the robot forward 60 inches ONCE.
# Physically reset the robot between running different tests.
# ------------------------------------------------------------

# Global derivative gains used by autonomousPID (optional)
Kd_linear_global = 0.0
Kd_rotation_global = 0.0

# Simple test selector. Set TEST_TO_RUN to one of:
# "move60", "arc", "chain", "turn", "smooth", or None to disable.
TEST_TO_RUN = None

def test_move_60in(Kp_l, Kp_r, initialMaxSpeed=50, maxSpeed=90, label=""):
    """Run a single 60-inch move with given Kp values.
    Place the robot at the starting line, facing forward,
    then call this function from main() or the VEX console.
    """
    print("=== 60-inch test:", label, "Kp_l=", Kp_l, "Kp_r=", Kp_r, "===")
    # Use current inertial heading as the target heading
    target_heading_rad = inertialSensor.rotation() * pi / 180.0
    # time_out is in loop iterations (200 Hz * seconds)
    # 2000 iterations at 0.005s ~ 10 seconds max
    autonomousPID([60.0, target_heading_rad],
                  initialMaxSpeed, maxSpeed,
                  time_out=2000,
                  export_flag=1,
                  Kp_l=Kp_l,
                  Kp_r=Kp_r)

def test_kp_default():
    """Test with your typical Kp values (e.g. 4 for linear, 30 for rotation)."""
    global Kd_linear_global, Kd_rotation_global
    Kd_linear_global = 0.0
    Kd_rotation_global = 0.0
    test_move_60in(4.0, 30.0, label="Kp default")

def test_kp_more_heading():
    """Increase heading Kp slightly to see if drift improves."""
    global Kd_linear_global, Kd_rotation_global
    Kd_linear_global = 0.0
    Kd_rotation_global = 0.0
    test_move_60in(4.0, 36.0, label="Kp_rotation +20%")

def test_kd_linear():
    """Add a small linear derivative term to see if overshoot improves."""
    global Kd_linear_global, Kd_rotation_global
    Kd_linear_global = 0.1   # example starting value
    Kd_rotation_global = 0.0
    test_move_60in(4.0, 30.0, label="Kd_linear=0.1")

def test_kd_rotation():
    """Add a small rotational derivative term to damp heading oscillations."""
    global Kd_linear_global, Kd_rotation_global
    Kd_linear_global = 0.0
    Kd_rotation_global = 1.0   # example starting value
    test_move_60in(4.0, 30.0, label="Kd_rotation=1.0")

# ------------------------------------------------------------
# New quick profiles for field testing
# ------------------------------------------------------------

def test_arc_with_exit():
    """24\" forward while arcing to +45 deg, keeps rolling at 15%."""
    target_heading_rad = math.radians(45)
    autonomousPID(
        [24.0, target_heading_rad],
        initialMaxSpeedLimit=30,
        maxSpeedLimit=70,
        time_out=1200,
        export_flag=1,
        Kp_l=4.0,
        Kp_r=30.0,
        exit_velocity_pct=15.0,
        slow_down_distance=8.0,
        settle_error_dist=0.6,
        settle_error_heading=math.radians(2.0),
        settle_loops=6,
        stop_at_end=False,
        heading_guard_deg=45.0,
        heading_guard_floor=0.3,
    )

def test_two_leg_chain():
    """Forward 36\", mild heading hold, then immediate 18\" to new heading."""
    start_heading = inertialSensor.rotation() * pi / 180.0
    # Leg 1
    autonomousPID(
        [36.0, start_heading],
        initialMaxSpeedLimit=30,
        maxSpeedLimit=80,
        time_out=1500,
        export_flag=0,
        Kp_l=4.0,
        Kp_r=30.0,
        exit_velocity_pct=12.0,
        slow_down_distance=10.0,
        settle_error_dist=0.7,
        settle_error_heading=math.radians(2.0),
        settle_loops=6,
        stop_at_end=False,
        heading_guard_deg=20.0,
        heading_guard_floor=0.2,
    )
    # Leg 2 (finishes stopped)
    autonomousPID(
        [18.0, math.radians(30.0)],
        initialMaxSpeedLimit=30,
        maxSpeedLimit=60,
        time_out=1000,
        export_flag=0,
        Kp_l=4.0,
        Kp_r=30.0,
        exit_velocity_pct=0.0,
        slow_down_distance=6.0,
        settle_error_dist=0.6,
        settle_error_heading=math.radians(2.0),
        settle_loops=6,
        stop_at_end=True,
        heading_guard_deg=25.0,
        heading_guard_floor=0.2,
    )

def test_in_place_turn():
    """Zero-distance heading change to verify turn dynamics."""
    current_heading = inertialSensor.rotation() * pi / 180.0
    target_heading = current_heading + math.radians(90)
    autonomousPID(
        [0.0, target_heading],
        initialMaxSpeedLimit=20,
        maxSpeedLimit=50,
        time_out=900,
        export_flag=1,
        Kp_l=0.0,     # linear unused
        Kp_r=45.0,
        exit_velocity_pct=0.0,
        slow_down_distance=2.0,
        settle_error_dist=0.2,
        settle_error_heading=math.radians(1.0),
        settle_loops=8,
        stop_at_end=True,
        heading_guard_deg=0.0,  # guard off so only turn PID acts
        heading_guard_floor=0.0,
    )

def test_smooth_chain():
    """
    Three-leg sequence to assess between-move smoothness:
      1) 36\" forward, heading hold, exits at 15%.
      2) 24\" arc to +40°, exits at 12%.
      3) 18\" forward to +60°, stops.
    """
    start_heading = inertialSensor.rotation() * pi / 180.0

    # Leg 1: straight with exit velocity
    autonomousPID(
        [36.0, start_heading],
        initialMaxSpeedLimit=30,
        maxSpeedLimit=80,
        time_out=1500,
        export_flag=0,
        Kp_l=4.0,
        Kp_r=30.0,
        exit_velocity_pct=15.0,
        slow_down_distance=10.0,
        settle_error_dist=0.7,
        settle_error_heading=math.radians(2.0),
        settle_loops=6,
        stop_at_end=False,
        heading_guard_deg=15.0,
        heading_guard_floor=0.2,
    )

    # Leg 2: gentle arc
    autonomousPID(
        [24.0, math.radians(40.0)],
        initialMaxSpeedLimit=30,
        maxSpeedLimit=70,
        time_out=1200,
        export_flag=0,
        Kp_l=4.0,
        Kp_r=30.0,
        exit_velocity_pct=12.0,
        slow_down_distance=8.0,
        settle_error_dist=0.6,
        settle_error_heading=math.radians(2.0),
        settle_loops=6,
        stop_at_end=False,
        heading_guard_deg=35.0,
        heading_guard_floor=0.25,
    )

    # Leg 3: finish and stop
    autonomousPID(
        [18.0, math.radians(60.0)],
        initialMaxSpeedLimit=30,
        maxSpeedLimit=60,
        time_out=1000,
        export_flag=1,  # print final telemetry
        Kp_l=4.0,
        Kp_r=30.0,
        exit_velocity_pct=0.0,
        slow_down_distance=6.0,
        settle_error_dist=0.5,
        settle_error_heading=math.radians(1.8),
        settle_loops=6,
        stop_at_end=True,
        heading_guard_deg=25.0,
        heading_guard_floor=0.2,
    )


def run_selected_test(name):
    """Dispatch helper so tests can be triggered from when_started1."""
    n = (name or "").lower()
    if n == "move60":
        test_move_60in(4.0, 30.0)
    elif n == "arc":
        test_arc_with_exit()
    elif n == "chain":
        test_two_leg_chain()
    elif n == "turn":
        test_in_place_turn()
    elif n == "smooth":
        test_smooth_chain()
    else:
        print("No test selected or unknown name:", name)
