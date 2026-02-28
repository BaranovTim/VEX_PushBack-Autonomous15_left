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

class GameElementsPushBack:
    BLUE_BLOCK = 0
    RED_BLOCK = 1

class AiColors:
     RED = Colordesc(1, 231, 67, 144, 22, 0.56)
     BLUE = Colordesc(2, 50, 180, 218, 20, 0.44)

vision = AiVision(Ports.PORT2, AiVision.ALL_AIOBJS, AiColors.RED, AiColors.BLUE)

MIN_AREA = 3500   # Adjust if needed
detected = False  
apermanence = 0
detected_red = False
detected_blue = False

LOGGING_ENABLED = False  # Set to False to disable all console logging

def show_vision_reading():
    global detected, apermanence, detected_red, detected_blue, largest_object, color
    EMPTY_THRESHOLD = 6
    while True:
        controller_1.screen.clear_screen()

        objects = vision.take_snapshot(AiVision.ALL_AIOBJS)
        red = vision.take_snapshot(AiColors.RED)
        blue = vision.take_snapshot(AiColors.BLUE)

        if not vision.installed():
            brain.screen.print("no vision sensor")
        
        frame_has_object = False
        frame_has_red = False
        frame_has_blue = False
        largest_object = None
        largest_area = 0

        if objects:
            for obj in objects:
                if obj.area < MIN_AREA:
                    continue

                # Track largest object
                if obj.area > largest_area:
                    largest_area = obj.area
                    largest_object = obj

                if obj.id == GameElementsPushBack.BLUE_BLOCK:
                    controller_1.screen.print(
                        "Blue block detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
                    )
                    if LOGGING_ENABLED:
                        print(
                            "Blue block detected at x: {}, y: {}, width: {}, height: {}".format(obj.centerX, obj.centerY, obj.width, obj.height)
                        )
                    brain.screen.new_line()
                    frame_has_blue = True

                if obj.id == GameElementsPushBack.RED_BLOCK:
                    controller_1.screen.print(
                        "Red block detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
                    )
                    if LOGGING_ENABLED:
                        print(
                            "Red block detected at x: {}, y: {}, width: {}, height: {}".format(obj.centerX, obj.centerY, obj.width, obj.height)
                        )
                    brain.screen.new_line()
                    frame_has_red = True

                if obj.centerX < 65:
                    if LOGGING_ENABLED:
                        print("Ignoring object at x: {}, y: {} because it's too far left".format(obj.centerX, obj.centerY))
                    continue

                frame_has_object = True

        if red:
            for obj in red:
                if obj.area < MIN_AREA:
                    continue
                if obj.area > largest_area:
                    largest_area = obj.area
                    largest_object = obj
                controller_1.screen.print(
                    "Red detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
                )
                if LOGGING_ENABLED:
                    print(
                        "Red detected at x: {}, y: {}, width: {}, height: {}".format(obj.centerX, obj.centerY, obj.width, obj.height)
                    )
                controller_1.screen.new_line()
                if obj.centerX < 65:
                    if LOGGING_ENABLED:
                        print("Ignoring object at x: {}, y: {} because it's too far left".format(obj.centerX, obj.centerY))
                    continue
                frame_has_object = True
                frame_has_red = True

        if blue:
            for obj in blue:
                if obj.area < MIN_AREA:
                    continue
                if obj.area > largest_area:
                    largest_area = obj.area
                    largest_object = obj
                controller_1.screen.print(
                    "Blue detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
                )
                if LOGGING_ENABLED:
                    print(
                        "Blue detected at x: {}, y: {}, width: {}, height: {}".format(obj.centerX, obj.centerY, obj.width, obj.height)
                    )
                controller_1.screen.new_line()

                if obj.centerX < 65:
                    if LOGGING_ENABLED:
                        print("Ignoring object at x: {}, y: {} because it's too far left".format(obj.centerX, obj.centerY))
                    continue
                frame_has_object = True
                frame_has_blue = True

        if frame_has_object:
            apermanence = 0
            detected = True
        if not frame_has_object:
            apermanence += 1
        else:
            apermanence = 0
        if apermanence >= EMPTY_THRESHOLD:
            detected = False
        
        detected_red = frame_has_red
        detected_blue = frame_has_blue
        
        # Print largest object info
        if largest_object:
            color = "RED" if largest_object.id == GameElementsPushBack.RED_BLOCK else "BLUE"
            controller_1.screen.print("Largest: {} ({})".format(color, largest_area))
            if LOGGING_ENABLED:
                print("Largest object: {} with area {}".format(color, largest_area))
        
        if LOGGING_ENABLED:
            print(detected)
            print(apermanence)
        controller_1.screen.print(detected)
        controller_1.screen.new_line() 
        wait(135, MSEC)


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
    Thread(show_vision_reading)
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

def arm_down():
    wait(0.5, SECONDS)
    sorter.set(True)

#SWITCH to autonomous() from user_control() -----------------------------------------------------------------------------
def autonomous():
    imu.set_rotation(0, DEGREES)
    #initial settings
    bunny_ear.set(False)
    mid_motor.set_velocity(100, PERCENT)
    top_motor.set_velocity(55, PERCENT)
    smooth_acceleration(70, 240, start_speed=30)
    turn_by(-49)                    
    #collecting the middle blocks
    mid_motor.spin(FORWARD)
    Thread(arm_down)
    smooth_acceleration(50, 639, end_speed=10)
    wait(0.5, SECONDS)
    smooth_acceleration(50, -100)
    
    #aligning to get the blocks under the long goal and collecting 'em  
    #turn_by(-3) 
    #mid_motor.spin(REVERSE)
    #smooth_acceleration(FORWARD, 620, MM)
    #wait(0.5, SECONDS)
    #mid_motor.stop()
    #sorter.set(False)

    #going back to the middle goal
    turn_by(-83)
    left_drive.set_velocity(70, PERCENT) 
    right_drive.set_velocity(70, PERCENT)
    smooth_acceleration(60, -385)
    
    #scoring into the middle goal
    mid_motor.spin(FORWARD)
    top_motor.spin(REVERSE)
    wait(1, SECONDS)
    top_motor.set_velocity(100, PERCENT)
    sorter.set(False)
    top_motor.stop()
    turn_by(3)
    left_drive.set_velocity(70, PERCENT) 
    right_drive.set_velocity(70, PERCENT)

    #thats the code for the long goal
    smooth_acceleration(100, 1180)
    turn_by(-49)
    # left_drive.set_velocity(30, PERCENT) 
    # right_drive.set_velocity(30, PERCENT)
    sorter.set(True)
    mid_motor.spin(FORWARD)
    # straight_heading = imu.heading()

    #collecting the blocks from the loader
    smooth_acceleration(40, 410, end_speed=40)
    start_time = time.time()
    while time.time() - start_time < 1.2: 
        wait(150, MSEC)
        smooth_acceleration(30, 10, end_speed=40)
        wait(150, MSEC)
        smooth_acceleration(-20, 10, end_speed=40) # Run for 3 seconds
    # if largest_object: 
    #     team_color = color
    #     print("Team color detected:", team_color)
    # while detected:
    #     wait(50, MSEC)
    #     smooth_acceleration(20, 10, end_speed=40)
    #     smooth_acceleration(-10, 10, end_speed=40)
    #     if largest_object:
    #         if color != team_color:
    #             break
    mid_motor.stop()
    # turn_to(straight_heading) #

    # left_drive.set_velocity(70, PERCENT) 
    # right_drive.set_velocity(70, PERCENT)

    smooth_acceleration(70, -350)
    turn_by(-4)
    smooth_acceleration(70, -350)
    # mid_motor.set_velocity(100, PERCENT)
    mid_motor.spin(FORWARD)
    top_motor.spin(REVERSE)
    wait(5, SECONDS)


    mid_motor.stop()
    top_motor.stop()
    drivetrain.stop()
    #going to the top left long goal
    

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