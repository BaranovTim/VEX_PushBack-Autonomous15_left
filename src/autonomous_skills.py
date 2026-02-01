from vex import *
import time

brain=Brain()
controller_1 = Controller(PRIMARY)
# hi

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





def smooth_acceleration(input_speed, distance):
    accel_dist = 0.2 * distance 
    # thats the distance that the robot accelerates/decelerates
    decel_start = 0.8 * distance 
    # thats when the robot starts decelerating
    step = 1 
    traveled = 0

    while traveled < distance:
        if traveled < accel_dist:
            speed = input_speed * (traveled / accel_dist)

        elif traveled > decel_start:
            speed = input_speed * ((distance - traveled) / (distance - decel_start))

        else:
            speed = input_speed

        left_drive.set_velocity(speed, PERCENT) 
        right_drive.set_velocity(speed, PERCENT)
        drivetrain.drive_for(FORWARD, step, MM)

        traveled += step


def smooth_input(value, deadband=10, expo=0.35, scale=1.0):
    # expo: 0 = linear, 1 = very soft center
    if abs(value) < deadband:
        return 0
    x = value / 100.0
    y = (1 - expo) * x + expo * (x * x * x)
    return y * 100 * scale


def pre_autonomous():
    brain.screen.clear_screen()
    calibrate_imu()
    imu.set_rotation(0, DEGREES)
    sorter.set(True)
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

bunny_ear_state = False
sorter_state = False
double_parking_state = False

def autonomous():
    Thread(show_heading)   
    sorter.set(True)
    double_parking.set(False)

    jitter = 15

    #initial settings
    mid_motor.set_velocity(100, PERCENT)
    top_motor.set_velocity(100, PERCENT)
    left_drive.set_velocity(20, PERCENT) 
    right_drive.set_velocity(20, PERCENT)
    drivetrain.set_turn_velocity(30, PERCENT)

    #aligning to get the middle blocks
    drivetrain.drive_for(FORWARD, 380, MM)  
    turn_by(-45)
    left_drive.set_velocity(30, PERCENT) 
    right_drive.set_velocity(30, PERCENT)                    
    #collecting the middle blocks
    mid_motor.spin(REVERSE)
    drivetrain.drive_for(FORWARD, 500, MM)
    wait(0.5 , SECONDS)
    drivetrain.drive_for(REVERSE, 100, MM)
    
    #aligning to get the blocks under the long goal and collecting 'em  
    #turn_by(-3) 
    #mid_motor.spin(REVERSE)
    #drivetrain.drive_for(FORWARD, 620, MM)
    #wait(0.5, SECONDS)
    #mid_motor.stop()
    #sorter.set(False)

    #going back to the middle goal
    turn_by(-86)
    left_drive.set_velocity(70, PERCENT) 
    right_drive.set_velocity(70, PERCENT)
    drivetrain.drive_for(REVERSE, 385, MM)
    
    #scoring into the middle goal
    mid_motor.spin(REVERSE)
    top_motor.spin(REVERSE)
    wait(2.5, SECONDS)
    top_motor.stop()
    sorter.set(True)
    turn_by(5)
    left_drive.set_velocity(70, PERCENT) 
    right_drive.set_velocity(70, PERCENT)

    #thats the code for the long goal
    drivetrain.drive_for(FORWARD, 1200, MM)
    turn_by(-49)
    left_drive.set_velocity(30, PERCENT) 
    right_drive.set_velocity(30, PERCENT)
    sorter.set(False)
    drivetrain.drive_for(FORWARD, 320, MM)

    start_time = time.time()
    while time.time() - start_time < 2:
        drivetrain.turn_for(LEFT, jitter, DEGREES)
        wait(0.1, SECONDS)
        drivetrain.turn_for(RIGHT, jitter, DEGREES)
        wait(0.1, SECONDS)
        drivetrain.drive_for(FORWARD, 20, MM)
    mid_motor.stop()

    left_drive.set_velocity(70, PERCENT) 
    right_drive.set_velocity(70, PERCENT)

    drivetrain.drive_for(REVERSE, 730, MM)
    mid_motor.set_velocity(100, PERCENT)
    mid_motor.spin(REVERSE)

    top_motor.spin(REVERSE)
    wait(5, SECONDS)

    #Second part
    left_drive.set_velocity(70, PERCENT) 
    right_drive.set_velocity(70, PERCENT)
    drivetrain.drive_for(FORWARD, 400, MM)
    imu.set_rotation(-175, DEGREES)
    drivetrain.turn_for(LEFT, 45, DEGREES)
    drivetrain.drive_for(FORWARD, 500, MM)
    sorter.set(True)
    right_drive.spin_for(FORWARD, 400)
    left_drive.set_velocity(71, PERCENT) 
    right_drive.set_velocity(70, PERCENT)
    drivetrain.drive(FORWARD)
    mid_motor.spin(REVERSE)
    top_motor.spin(REVERSE)
    wait(1.2, SECONDS)
    drivetrain.stop()
    sorter.set(True)

    wait(6.7, SECONDS)
    mid_motor.stop()
    top_motor.stop()

autonomous()