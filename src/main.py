from vex import *
import time

brain=Brain()
controller_1 = Controller(PRIMARY)


# Motors
top_motor = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False) # type of gear (medium)
mid_motor = Motor(Ports.PORT5, GearSetting.RATIO_18_1, False)

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


drivetrain = DriveTrain(left_drive, right_drive, 314, 360, 40, MM, 1) # CHECK THIS (wheel travel per 1 cycle, track width, distance between front and back wheels, units, gear ratio )
   
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

def turn_to(target_deg, kP=1.2, min_speed=8, max_speed=60, tol=1.5):
    while True:
        err = target_deg - imu.rotation(DEGREES)

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

        wait(20, MSEC)

    left_drive.stop()
    right_drive.stop()


def turn_by(delta_deg, **kwargs):
    target = imu.rotation(DEGREES) + delta_deg
    turn_to(target, **kwargs)




def smooth_input(value, deadband=10, expo=0.35, scale=1.0):
    # expo: 0 = linear, 1 = very soft center
    if abs(value) < deadband:
        return 0
    x = value / 100.0
    y = (1 - expo) * x + expo * (x * x * x)
    return y * 100 * scale

def pre_autonomous():
    brain.screen.clear_screen()
    brain.screen.print("pre auton code")
    calibrate_imu()
    imu.set_rotation(0, DEGREES)
    sorter.set(True)
    double_parking.set(False)
    wait(1, SECONDS)                                                                                                                                                                         



bunny_ear_state = False
sorter_state = False
double_parking_state = False

#SWITCH to autonomous() from user_control()
def user_control():
    brain.screen.print("autonomous code")
    sorter.set(True)
    double_parking.set(False)

    jitter = 15

    #initial settings
    mid_motor.set_velocity(100, PERCENT)
    left_drive.set_velocity(20, PERCENT) # 10 is set just for testing
    right_drive.set_velocity(20, PERCENT) # same here
    drivetrain.set_turn_velocity(30, PERCENT)

    #aligning to get the middle blocks
    drivetrain.drive_for(FORWARD, 380, MM)
    turn_by(-45)                            #----------------- tested only up to here.. everything further has not been tested
    
    #collecting the middle blocks
    mid_motor.spin(REVERSE)
    drivetrain.drive_for(FORWARD, 400, MM)
    mid_motor.stop()

    #aligning to get the blocks under the long goal and collecting 'em
    turn_by(-10)
    mid_motor.spin(REVERSE)
    drivetrain.drive_for(FORWARD, 500, MM)
    wait(0.5, SECONDS)
    mid_motor.stop()

    #going back to the middle goal
    #drivetrain.drive_for(REVERSE, 400, MM)
    #turn_by(20)
    #drivetrain.drive_for(REVERSE, 400, MM)
    #turn_by(-90)
    #drivetrain.drive_for(REVERSE, 200, MM, wait=False)
    
    #scoring into the middle goal
    #mid_motor.spin(REVERSE)
    #top_motor.spin(REVERSE)
    #wait(2, SECONDS)


    #thats the code for the long goal
    #mid_motor.spin(REVERSE)
    #start_time = time.time()
    #left_drive.set_velocity(50, PERCENT)
    #ight_drive.set_velocity(50, PERCENT)
    #while time.time() - start_time < 5:
    #    drivetrain.turn_for(LEFT, jitter, DEGREES)
    #    wait(0.1, SECONDS)
    #    drivetrain.turn_for(RIGHT, jitter, DEGREES)
    #    wait(0.1, SECONDS)
    #    drivetrain.drive_for(FORWARD, 20, MM)

   
    mid_motor.stop()
    drivetrain.stop()

def mix_arcade(forward, turn): # makes the robot to be able to turn and go forward at the same time
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

#SWITCH to user_control() from autonomous()
def autonomous():
    global bunny_ear_state, sorter_state, double_parking_state
    brain.screen.clear_screen()
    top_motor.set_stopping(HOLD) # tries to freeze when stops
    mid_motor.set_stopping(HOLD)
    left_motor1.set_stopping(COAST)
    left_motor2.set_stopping(COAST)
    right_motor1.set_stopping(COAST)
    right_motor2.set_stopping(COAST)
    left_drive.set_stopping(COAST)
    right_drive.set_stopping(COAST)


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
        turn = raw_turn * 1

        # turn kick based on driver "jerk", but only when actually moving
        dturn = raw_turn - prev_turn
        prev_turn = raw_turn

        if abs(dturn) > 25 and abs(forward) > 15:
            turn = turn * 1.25

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
            mid_motor.spin(FORWARD)
        elif controller_1.buttonL1.pressing():
            mid_motor.set_velocity(100, PERCENT)
            mid_motor.spin(REVERSE)
        else:
            mid_motor.stop()

        wait(20, MSEC)


# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()