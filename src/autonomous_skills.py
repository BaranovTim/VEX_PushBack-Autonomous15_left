from vex import *
import time

brain=Brain()
controller_1 = Controller(PRIMARY)


# Motors
top_motor = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False) # type of gear (medium)
mid_motor1 = Motor(Ports.PORT5, GearSetting.RATIO_18_1, False)
mid_motor2 = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
mid_motor = MotorGroup(mid_motor1, mid_motor2)

left_motor1 = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
left_motor2 = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
left_drive = MotorGroup(left_motor1, left_motor2)

right_motor1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
right_motor2 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
right_drive = MotorGroup(right_motor1, right_motor2)


sorter = DigitalOut(brain.three_wire_port.a)
bunny_ear = DigitalOut(brain.three_wire_port.b)
double_parking = DigitalOut(brain.three_wire_port.c)


imu = Inertial(Ports.PORT7)


drivetrain = DriveTrain(left_drive, right_drive, 314, 360, 40, MM, 1) 
# CHECK THIS (wheel travel per 1 cycle, track width, distance between front and back wheels, units, gear ratio )


# /// PID

def pid(target_mm, kP=0.25, kI=0.0, kD=0.15,
                         max_power=80, settle_error=5, settle_time_ms=250, timeout_ms=4000):

    left_drive.reset_position()
    right_drive.reset_position()

    integral = 0.0
    last_error = 0.0
    dt = 0.02 

    settled_ms = 0
    elapsed_ms = 0

    while elapsed_ms < timeout_ms:
        
        left_deg = left_drive.position(DEGREES)
        right_deg = right_drive.position(DEGREES)
        avg_deg = (left_deg + right_deg) / 2.0

        wheel_circ_mm = 319.19  
        traveled_mm = (avg_deg / 360.0) * wheel_circ_mm

        error = target_mm - traveled_mm

        
        integral += error * dt
        if integral > 2000: integral = 2000
        if integral < -2000: integral = -2000

        derivative = (error - last_error) / dt
        last_error = error

        output = (kP * error) + (kI * integral) + (kD * derivative)

        
        if output > max_power: output = max_power
        if output < -max_power: output = -max_power

        drivetrain.set_drive_velocity(abs(output), PERCENT)

        if output >= 0:
            drivetrain.drive(FORWARD)
        else:
            drivetrain.drive(REVERSE)

        
        if abs(error) <= settle_error:
            settled_ms += int(dt * 1000)
            if settled_ms >= settle_time_ms:
                break
        else:
            settled_ms = 0

        wait(int(dt * 1000), MSEC)
        elapsed_ms += int(dt * 1000)

    drivetrain.stop(BRAKE)




# /// INERTIAL SENSOR 

def calibrate_imu():
    imu.calibrate()
    while imu.is_calibrating():
        wait(20, MSEC)

def wrap180(deg):
    # convert any angle difference to [-180, 180]
    while deg > 180: deg -= 360
    while deg < -180: deg += 360
    return deg

def turn_to(target_deg, kP=1.5, min_speed=8, max_speed=50, tol=1.5):
    while True:
        err = wrap180(target_deg - imu.rotation(DEGREES))

        if abs(err) <= tol:
            break

        speed = kP * err
        speed = max(min(speed, max_speed), -max_speed)

        if 0 < speed < min_speed: speed = min_speed
        if -min_speed < speed < 0: speed = -min_speed

        left_drive.set_velocity(speed, PERCENT)
        right_drive.set_velocity(-speed, PERCENT)
        left_drive.spin(FORWARD)
        right_drive.spin(FORWARD)

        wait(0, MSEC)

    left_drive.stop()
    right_drive.stop()

    left_drive.set_velocity(70, PERCENT)
    right_drive.set_velocity(70, PERCENT)

def show_heading():
    controller_1.screen.clear_screen()
    while True:
        heading = imu.rotation(DEGREES)
        controller_1.screen.set_cursor(1, 1)
        controller_1.screen.print("Heading:")
        controller_1.screen.set_cursor(2, 1)
        controller_1.screen.print(round(heading, 1))
        wait(500, MSEC)

def turn_by(delta_deg, **kwargs):
    target = imu.rotation(DEGREES) + delta_deg
    turn_to(target, **kwargs) 

# -- end of the code for Inertial Sensor

def turn_one_side(deg=90, side=RIGHT, speed=60):

    left_drive.stop()
    right_drive.stop()

    if side == LEFT:
        left_drive.set_velocity(0, PERCENT)
        right_drive.set_velocity(speed, PERCENT)
        right_drive.spin_for(FORWARD, deg, DEGREES)

    elif side == RIGHT:
        right_drive.set_velocity(0, PERCENT)
        left_drive.set_velocity(speed, PERCENT)
        left_drive.spin_for(FORWARD, deg, DEGREES)

    else:
        controller_1.screen.clear_screen()
        controller_1.screen.print("Invalid side")



def smooth_input(value, deadband=10, expo=0.35, scale=1.0):
    # expo: 0 = linear, 1 = very soft center
    if abs(value) < deadband:
        return 0
    x = value / 100.0
    y = (1 - expo) * x + expo * (x * x * x)
    return y * 100 * scale

def smooth_acceleration(input_speed, distance_mm, start_speed=30, end_speed=20):
    if distance_mm == 0:
        left_drive.stop()
        right_drive.stop()
        return

    # Decide direction from sign of distance
    dir = FORWARD if distance_mm > 0 else REVERSE
    target = abs(distance_mm)

    accel_dist  = 0.2 * target
    decel_start = 0.8 * target

    left_drive.reset_position()
    right_drive.reset_position()

    # Start moving
    left_drive.spin(dir, start_speed, PERCENT)
    right_drive.spin(dir, start_speed, PERCENT)

    while True:
        traveled_deg = (abs(left_drive.position(DEGREES)) + abs(right_drive.position(DEGREES))) / 2
        traveled_mm  = traveled_deg * (314 / 360)  # your conversion

        if traveled_mm >= target:
            break

        if traveled_mm < accel_dist:
            ratio = traveled_mm / accel_dist if accel_dist > 0 else 1
            speed = max(start_speed, input_speed * ratio)

        elif traveled_mm > decel_start:
            denom = (target - decel_start)
            ratio = (target - traveled_mm) / denom if denom > 0 else 0
            speed = max(end_speed, input_speed * ratio)

        else:
            speed = input_speed

        # Force direction explicitly every loop
        left_drive.spin(dir, speed, PERCENT)
        right_drive.spin(dir, speed, PERCENT)

        wait(10, MSEC)

    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)


def pre_autonomous():
    brain.screen.clear_screen()
    controller_1.screen.clear_screen()
    imu.set_rotation(0, DEGREES)
    bunny_ear.set(True)
    double_parking.set(False)
    wait(2, SECONDS)      
                                                                                                                                                                    
def mid_motor_break():
    time = 0.5
    mid_motor.set_velocity(50, PERCENT)
    for i in range(3): 
        mid_motor.spin(REVERSE)
        wait(time, SECONDS)
        mid_motor.spin(FORWARD)
        wait(time, SECONDS)
        time += 0.3


def mix_arcade(forward, turn): 
    # makes the robot to be able to turn and go forward at the same time
    left = forward + turn
    right = forward - turn

    m = max(abs(left), abs(right))
    if m > 100:
        scale = 100 / m
        left *= scale
        right *= scale
    return left, right

def deadband(v, db=10):
    return 0 if abs(v) < db else v

bunny_ear_state = False
sorter_state = False
double_parking_state = False

#SWITCH to autonomous() from user_control() -----------------------------------------------------------------------------
def autonomous():
    imu.set_rotation(0, DEGREES)
    #initial settings
    bunny_ear.set(False)
    mid_motor.set_velocity(100, PERCENT)
    top_motor.set_velocity(100, PERCENT)

    #driving to the first loader
    smooth_acceleration(90, 1185, start_speed=50, end_speed=40)
    sorter.set(True)
    turn_by(-85)
    mid_motor.spin(FORWARD)
    straight_heading = imu.heading()
    controller_1.screen.print(straight_heading)

    #collecting the blocks from the loader
    smooth_acceleration(40, 315, end_speed=40)
    wait(2.5, SECONDS)
    mid_motor.stop()
    #turn_to(straight_heading) #smartttttttttttttt

    #going to the long goal and scoring the blocks
    smooth_acceleration(70, -270, start_speed=40, end_speed=40)
    turn_by(-40)
    smooth_acceleration(70, -545, start_speed=40, end_speed=40)
    turn_by(40)
    #straight_heading = imu.heading()
    smooth_acceleration(100, -1670, end_speed=40, start_speed=30)
    turn_by(50)
    smooth_acceleration(70, -270, start_speed=50, end_speed=40)
    turn_by(120)
    current_heading = imu.heading()
    controller_1.screen.print(current_heading)
    '''
    is_imu_right = False
    if abs(straight_heading) - abs(current_heading) <= 2 and abs(straight_heading) - abs(current_heading) >= -2:
        turn_to(straight_heading)
        is_imu_right = True
    '''
    
    smooth_acceleration(70, -570)
    mid_motor.spin(FORWARD)
    top_motor.spin(REVERSE) # hi tim
    wait(3.5, SECONDS)
    top_motor.stop()
    
    #finished scoring the blocks from the first loader
    #2nd loader
    '''
    turn_by(2)
    if is_imu_right == True:
        turn_to(straight_heading)
    '''
    smooth_acceleration(40, 725, end_speed=40)
    wait(3.5, SECONDS)
    mid_motor.stop()
    smooth_acceleration(80, -725, end_speed=40, start_speed=70)
    mid_motor.spin(FORWARD)
    top_motor.spin(REVERSE)
    wait(3.5, SECONDS)
    mid_motor.stop()
    top_motor.stop()

    #going to the third loader
    smooth_acceleration(80, 350, start_speed=70, end_speed=50)
    turn_by(87)
    smooth_acceleration(100, 2360, end_speed=40, start_speed=70)
    #------------------------------------------------------------------------------------------------
    turn_by(-85)
    sorter.set(True)
    mid_motor.spin(FORWARD)
    straight_heading = imu.heading()

    #collecting the blocks from the loader
    smooth_acceleration(40, 280, end_speed=40)
    wait(2.5, SECONDS)
    mid_motor.stop()
    #turn_to(straight_heading) #smartttttttttttttt

    #going to the long goal
    smooth_acceleration(70, -270, start_speed=40, end_speed=40)
    turn_by(-40)
    smooth_acceleration(50, -615, start_speed=40, end_speed=40)
    turn_by(40)
    smooth_acceleration(100, -1670, end_speed=40, start_speed=60)
    turn_by(50)
    smooth_acceleration(50, -270, start_speed=40, end_speed=40)
    turn_by(120)
    
    smooth_acceleration(70, -585, start_speed=40)
    mid_motor.spin(FORWARD)
    top_motor.spin(REVERSE) # hi tim
    wait(3.5, SECONDS)
    top_motor.stop()
    
    #finished scoring the blocks from the third loader
    #4th loader
    smooth_acceleration(40, 710, end_speed=40)
    wait(3.5, SECONDS)
    mid_motor.stop()
    smooth_acceleration(80, -710, end_speed=40, start_speed=70)
    mid_motor.spin(FORWARD)
    top_motor.spin(REVERSE)
    wait(3.5, SECONDS)
    mid_motor.stop()
    top_motor.stop()
    
    #going to the parking zone
    smooth_acceleration(80, 350, start_speed=60, end_speed=60)
    turn_by(87)
    smooth_acceleration(100, 1175, start_speed=70, end_speed=70)
    turn_by(-90)
    drivetrain.drive(FORWARD)
    wait(1.2, SECONDS)
    drivetrain.stop()
    double_parking.set(True)
    drivetrain.drive_for(REVERSE, 30, MM)

#SWITCH to user_control() from autonomous() -----------------------------------------------------------------------------------
def user_control():
    global bunny_ear_state, sorter_state, double_parking_state
    brain.screen.clear_screen()
    top_motor.set_stopping(HOLD) # tries to freeze when stops
    mid_motor.set_stopping(HOLD)
    left_motor1.set_stopping(HOLD)
    left_motor2.set_stopping(HOLD)
    right_motor1.set_stopping(HOLD)
    right_motor2.set_stopping(HOLD)
    left_drive.set_stopping(HOLD)
    right_drive.set_stopping(HOLD)


    prevA = False
    prevB = False
    prevY = False
    prev_turn = 0

    while True:

        #-------- Driving
        raw_forward = deadband(controller_1.axis3.position(), 10)
        raw_turn    = deadband(controller_1.axis1.position(), 10)

        # cubic forward
        forward = smooth_input(raw_forward)

        # linear turn (tune 0.9..1.2)
        turn = smooth_input(raw_turn)

        left_speed, right_speed = mix_arcade(forward, turn)
        
        left_drive.set_velocity(left_speed, PERCENT) 
        right_drive.set_velocity(right_speed, PERCENT) 
        
        left_drive.spin(FORWARD) 
        right_drive.spin(FORWARD)

        A = controller_1.buttonA.pressing()
        if A and not prevA:
            bunny_ear_state = not bunny_ear_state
            bunny_ear.set(bunny_ear_state)
        prevA = A
        
        B = controller_1.buttonB.pressing()
        if B and not prevB:
            sorter_state = not sorter_state
            sorter.set(sorter_state)
        prevB = B

        Y = controller_1.buttonY.pressing()
        if Y and not prevY:
            double_parking_state = not double_parking_state
            double_parking.set(double_parking_state)
        prevY = Y

        #-------- Motor 1
        if controller_1.buttonR2.pressing():
            top_motor.set_velocity(100, PERCENT)
            top_motor.spin(FORWARD)
            
        elif controller_1.buttonR1.pressing():
            top_motor.set_velocity(100, PERCENT)
            top_motor.spin(REVERSE)
        else:
            top_motor.stop()

        #-------- Motor 2
        if controller_1.buttonL2.pressing():
            mid_motor.set_velocity(100, PERCENT)
            mid_motor.spin(REVERSE)
        elif controller_1.buttonL1.pressing():
            mid_motor.set_velocity(100, PERCENT)
            mid_motor.spin(FORWARD)
        else:
            mid_motor.stop()

        wait(20, MSEC)


# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()