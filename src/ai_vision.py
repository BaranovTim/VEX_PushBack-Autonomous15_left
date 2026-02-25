from vex import *

brain = Brain()

class GameElementsPushBack:
    BLUE_BLOCK = 0
    RED_BLOCK = 1

class AiColors:
     RED = Colordesc(1, 231, 67, 144, 22, 0.56)
     BLUE = Colordesc(2, 50, 180, 218, 20, 0.44)

vision = AiVision(Ports.PORT2, AiVision.ALL_AIOBJS, AiColors.RED, AiColors.BLUE)

MIN_AREA = 100   # Adjust if needed

while True:
    brain.screen.render()
    brain.screen.clear_screen()

    objects = vision.take_snapshot(AiVision.ALL_AIOBJS)
    red = vision.take_snapshot(AiColors.RED)
    blue = vision.take_snapshot(AiColors.BLUE)

    if not vision.installed():
        brain.screen.print("no vision sensor")

    detected = False

    if objects:
        for obj in objects:
            if obj.area < MIN_AREA:
                continue

            detected = True

            if obj.id == GameElementsPushBack.BLUE_BLOCK:
                brain.screen.print(
                    "Blue block detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
                )
                print(
                    "Blue block detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
                )
                brain.screen.new_line()

            if obj.id == GameElementsPushBack.RED_BLOCK:
                brain.screen.print(
                    "Red block detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
                )
                print(
                    "Red block detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
                )
                brain.screen.new_line()

    if red:
        for obj in red:
            if obj.area < MIN_AREA:
                continue

            detected = True

            brain.screen.print(
                "Red detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
            )
            print(
                "Red detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
            )
            brain.screen.new_line()

    if blue:
        for obj in blue:
            if obj.area < MIN_AREA:
                continue

            detected = True

            brain.screen.print(
                "Blue detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
            )
            print(
                "Blue detected at x: {}, y: {}".format(obj.centerX, obj.centerY)
            )
            brain.screen.new_line()

    if not detected:
        brain.screen.print("No objects")
        print("No objects")

    wait(500, MSEC)