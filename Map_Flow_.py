import utils.session
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Stop, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch


# ============================================================
# WRO 2026 SENIOR - MOSAIC MASTER
# SLIDE-CARRIER VERSION
#
# Correct strategy:
# 1. Scan mosaic in serpentine order
# 2. Collect all required tiles from source to source
# 3. Store tiles into slope slide carrier
# 4. Door holds tiles at the end
# 5. Go to mosaic frame
# 6. Move robot in serpentine order
# 7. Open door one-by-one to release tiles
# ============================================================


# ============================================================
# 1. PORT SETUP
# CHANGE PORTS BASED ON YOUR ROBOT
# ============================================================

hub = PrimeHub()

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)

# Combined grab + lift mechanism
grab_lift_motor = Motor(Port.C)

# Scanner slider motor
scanner_motor = Motor(Port.D)

# Door motor for slide carrier
door_motor = Motor(Port.E)

# One color sensor for line + intersection + mosaic scan
color_sensor = ColorSensor(Port.F)

robot = DriveBase(
    left_motor,
    right_motor,
    wheel_diameter=56,
    axle_track=120
)


# ============================================================
# 2. TUNING VALUES
# ============================================================

DRIVE_SPEED = 250
LINE_SPEED = 130
TURN_SPEED = 160

BLACK_THRESHOLD = 35
LINE_KP = 1.2

TURN_KP = 3.0
TURN_TOLERANCE = 2

INTERSECTION_BLACK_TIME = 120
AFTER_INTERSECTION_FORWARD = 70

# Scanner positions for 12 serpentine scan locations.
# Tune these based on your scanner mechanism.
SCAN_POSITIONS = [
    0, 120, 240, 360,
    480, 600, 720, 840,
    960, 1080, 1200, 1320
]

# Serpentine placement order:
#
# [0] [1] [2] [3]
# [4] [5] [6] [7]
# [8] [9] [10] [11]
#
# Robot scans / places:
# 0 → 1 → 2 → 3
# 7 → 6 → 5 → 4
# 8 → 9 → 10 → 11

SERPENTINE_SLOTS = [
    0, 1, 2, 3,
    7, 6, 5, 4,
    8, 9, 10, 11
]

# Door mechanism angles
DOOR_CLOSED_ANGLE = 0
DOOR_OPEN_ANGLE = 65

# Grab/lift mechanism angles
GRAB_OPEN_ANGLE = 0
GRAB_CLOSED_ANGLE = 80
GRAB_LIFT_ANGLE = 150
GRAB_DROP_TO_SLIDE_ANGLE = 30

# Placement movement tuning
# These are physical movement distances between mosaic cells.
# Tune on the real frame.
MOSAIC_CELL_STEP_MM = 55
MOSAIC_ROW_STEP_MM = 55


# ============================================================
# 3. BASIC MOVEMENT
# ============================================================

def reset_gyro():
    hub.imu.reset_heading(0)
    wait(300)


def gyro_turn(target_angle):
    while True:
        current_angle = hub.imu.heading()
        error = target_angle - current_angle

        if abs(error) <= TURN_TOLERANCE:
            break

        turn_rate = error * TURN_KP

        if turn_rate > TURN_SPEED:
            turn_rate = TURN_SPEED
        elif turn_rate < -TURN_SPEED:
            turn_rate = -TURN_SPEED

        robot.drive(0, turn_rate)
        wait(10)

    robot.stop()
    wait(150)


def drive_mm(distance, speed=DRIVE_SPEED):
    robot.settings(straight_speed=speed)
    robot.straight(distance)
    wait(100)


def is_black():
    return color_sensor.reflection() < BLACK_THRESHOLD


def drive_until_black(speed=120):
    robot.drive(speed, 0)

    while not is_black():
        wait(10)

    robot.stop()
    wait(100)


def drive_past_intersection():
    drive_mm(AFTER_INTERSECTION_FORWARD, 120)


# ============================================================
# 4. LINE FOLLOWING + INTERSECTION DETECTION
# ============================================================

def follow_line_until_intersections(target_count, base_speed=LINE_SPEED):
    count = 0
    black_timer = StopWatch()
    on_intersection = False

    while count < target_count:
        reflection = color_sensor.reflection()
        error = BLACK_THRESHOLD - reflection
        correction = error * LINE_KP

        robot.drive(base_speed, correction)

        if is_black():
            if not on_intersection:
                black_timer.reset()
                on_intersection = True

            elif black_timer.time() > INTERSECTION_BLACK_TIME:
                count += 1
                robot.stop()
                wait(100)

                drive_past_intersection()
                on_intersection = False

        else:
            on_intersection = False

        wait(10)

    robot.stop()
    wait(100)


# ============================================================
# 5. MOSAIC SCANNING
# ============================================================

def read_color_name():
    detected = color_sensor.color()

    if detected == Color.YELLOW:
        return "yellow"
    elif detected == Color.BLUE:
        return "blue"
    elif detected == Color.GREEN:
        return "green"
    elif detected == Color.WHITE:
        return "white"

    # Backup for white if color detection is unstable
    if color_sensor.reflection() > 70:
        return "white"

    return "unknown"


def move_scanner_to_scan_index(scan_index):
    scanner_motor.run_target(
        speed=400,
        target_angle=SCAN_POSITIONS[scan_index],
        then=Stop.HOLD
    )
    wait(180)


def scan_mosaic_serpentine():
    mosaic_order = []

    for scan_index in range(12):
        move_scanner_to_scan_index(scan_index)

        scanned_color = read_color_name()
        mosaic_order.append(scanned_color)

        print("Scan", scan_index, "=", scanned_color)

    # Retract scanner
    scanner_motor.run_target(500, 0, then=Stop.HOLD)
    wait(300)

    return mosaic_order


# ============================================================
# 6. SLIDE CARRIER MECHANISM
# ============================================================

def close_door():
    door_motor.run_target(
        speed=300,
        target_angle=DOOR_CLOSED_ANGLE,
        then=Stop.HOLD
    )
    wait(120)


def open_door():
    door_motor.run_target(
        speed=300,
        target_angle=DOOR_OPEN_ANGLE,
        then=Stop.HOLD
    )
    wait(120)


def release_one_tile_from_slide():
    """
    Door opens shortly so only one tile exits.
    Tune the open time and door angle carefully.
    """
    open_door()
    wait(250)   # tune this so exactly one tile drops
    close_door()
    wait(150)


def open_grabber():
    grab_lift_motor.run_target(
        speed=400,
        target_angle=GRAB_OPEN_ANGLE,
        then=Stop.HOLD
    )
    wait(120)


def close_grabber():
    grab_lift_motor.run_target(
        speed=400,
        target_angle=GRAB_CLOSED_ANGLE,
        then=Stop.HOLD
    )
    wait(120)


def lift_grabbed_tile():
    grab_lift_motor.run_target(
        speed=400,
        target_angle=GRAB_LIFT_ANGLE,
        then=Stop.HOLD
    )
    wait(150)


def drop_tile_into_slide():
    """
    Tile is released into the sloped carrier.
    Door stays closed, so the tile waits in the queue.
    """
    grab_lift_motor.run_target(
        speed=350,
        target_angle=GRAB_DROP_TO_SLIDE_ANGLE,
        then=Stop.HOLD
    )
    wait(150)

    open_grabber()
    wait(150)

    # Return grabber to ready position
    grab_lift_motor.run_target(
        speed=350,
        target_angle=GRAB_OPEN_ANGLE,
        then=Stop.HOLD
    )
    wait(150)


def pick_tile_and_store_to_slide():
    """
    Full sequence:
    pick tile from source → lift → drop into slide carrier.
    """
    open_grabber()

    # approach tile
    drive_mm(45, 80)

    close_grabber()
    lift_grabbed_tile()

    # back away from source
    drive_mm(-45, 80)

    # store on top slide
    drop_tile_into_slide()


# ============================================================
# 7. START TO MOSAIC
# ============================================================

def go_start_to_mosaic_scan_position():
    print("Start → Mosaic scan position")

    reset_gyro()
    close_door()

    drive_mm(180, 180)
    drive_until_black(120)

    # Tune this based on the real map.
    follow_line_until_intersections(3)

    gyro_turn(0)
    drive_mm(120, 100)

    print("Arrived at mosaic scan position")


# ============================================================
# 8. SOURCE-TO-SOURCE NAVIGATION
# ============================================================
# Change SOURCE_LEVEL based on the real physical order of tile sources.
#
# Example assumption:
# top    = white
#        = green
#        = blue
# bottom = yellow
# ============================================================

SOURCE_LEVEL = {
    "white": 0,
    "green": 1,
    "blue": 2,
    "yellow": 3
}


def route_mosaic_to_source(color):
    print("Route: mosaic →", color)

    drive_mm(-100, 120)
    gyro_turn(-90)
    drive_until_black()

    # Number of intersections depends on color source level.
    # Tune these if needed.
    intersections = {
        "white": 1,
        "green": 2,
        "blue": 3,
        "yellow": 4
    }

    follow_line_until_intersections(intersections[color])

    # Face the source
    gyro_turn(-180)
    drive_mm(80, 90)


def move_one_source_down():
    """
    Move from one source to the next lower source.
    Example: white → green, green → blue, blue → yellow
    """
    print("Move one source down")

    gyro_turn(-180)
    drive_until_black()
    follow_line_until_intersections(1)

    gyro_turn(-180)
    drive_mm(70, 90)


def move_one_source_up():
    """
    Move from one source to the next upper source.
    Example: yellow → blue, blue → green, green → white
    """
    print("Move one source up")

    gyro_turn(0)
    drive_until_black()
    follow_line_until_intersections(1)

    gyro_turn(-180)
    drive_mm(70, 90)


def move_to_next_tile_same_source():
    """
    If the next required tile is the same color,
    do not leave the source. Just shift to the next tile.
    """
    print("Same source → move slightly to next tile")
    drive_mm(35, 70)


def route_source_to_source(current_color, target_color):
    print("Route:", current_color, "→", target_color)

    if current_color == target_color:
        move_to_next_tile_same_source()
        return

    current_level = SOURCE_LEVEL[current_color]
    target_level = SOURCE_LEVEL[target_color]

    difference = target_level - current_level

    if difference > 0:
        for i in range(difference):
            move_one_source_down()

    elif difference < 0:
        for i in range(abs(difference)):
            move_one_source_up()


def route_source_to_mosaic(current_color):
    print("Route:", current_color, "→ mosaic")

    # Face back toward main line
    gyro_turn(0)
    drive_until_black()

    intersections = {
        "white": 1,
        "green": 2,
        "blue": 3,
        "yellow": 4
    }

    follow_line_until_intersections(intersections[current_color])

    gyro_turn(90)
    drive_mm(120, 100)

    print("Arrived at mosaic placement area")


def go_to_location(current_pos, target_pos):
    """
    current_pos can be:
    'mosaic', 'yellow', 'blue', 'green', 'white'

    target_pos can be:
    'mosaic', 'yellow', 'blue', 'green', 'white'
    """

    if current_pos == target_pos:
        if target_pos != "mosaic":
            move_to_next_tile_same_source()
        return

    if current_pos == "mosaic" and target_pos in SOURCE_LEVEL:
        route_mosaic_to_source(target_pos)

    elif current_pos in SOURCE_LEVEL and target_pos in SOURCE_LEVEL:
        route_source_to_source(current_pos, target_pos)

    elif current_pos in SOURCE_LEVEL and target_pos == "mosaic":
        route_source_to_mosaic(current_pos)

    else:
        print("No route from", current_pos, "to", target_pos)


# ============================================================
# 9. COLLECT ALL TILES FIRST
# ============================================================

def collect_all_tiles_to_slide(pickup_order):
    """
    This is the key corrected part.

    Since the slide carrier releases FIRST-IN-FIRST-OUT,
    collect tiles in the same order as the scanned mosaic.
    """

    current_pos = "mosaic"

    close_door()

    for i in range(12):
        target_color = pickup_order[i]

        print("--------------------------------")
        print("Pickup number:", i + 1)
        print("Current position:", current_pos)
        print("Need color:", target_color)

        go_to_location(current_pos, target_color)

        pick_tile_and_store_to_slide()

        current_pos = target_color

    return current_pos


# ============================================================
# 10. MOSAIC PLACEMENT MOVEMENT
# ============================================================
# This assumes the robot places tiles by physically moving
# across the mosaic frame in serpentine order.
#
# Placement path:
#
# slot 0 → slot 1 → slot 2 → slot 3
#                              ↓
# slot 7 ← slot 6 ← slot 5 ← slot 4
# ↓
# slot 8 → slot 9 → slot 10 → slot 11
# ============================================================

def align_to_first_mosaic_slot():
    """
    Align robot so the slide door is positioned above / beside slot 0.
    Tune this on real robot.
    """
    print("Align to mosaic slot 0")

    gyro_turn(0)

    # Final alignment movement.
    # Tune based on your robot and frame.
    drive_mm(70, 80)


def move_to_next_mosaic_slot(place_index):
    """
    Moves robot from current serpentine placement position
    to the next serpentine position.

    place_index means the tile that was just placed:
    after placing index 0, move to index 1
    after placing index 1, move to index 2
    etc.
    """

    # Row 1: move right from index 0 → 1 → 2 → 3
    if place_index in [0, 1, 2]:
        move_one_slot_right()

    # End of row 1: move down to row 2
    elif place_index == 3:
        move_down_to_next_row()
        face_left_for_row_2()

    # Row 2: move left from index 4 → 5 → 6 → 7
    # Remember these are placement indexes, not real slot numbers.
    elif place_index in [4, 5, 6]:
        move_one_slot_left()

    # End of row 2: move down to row 3
    elif place_index == 7:
        move_down_to_next_row()
        face_right_for_row_3()

    # Row 3: move right from index 8 → 9 → 10 → 11
    elif place_index in [8, 9, 10]:
        move_one_slot_right()


def move_one_slot_right():
    print("Move one mosaic slot right")

    # If robot is facing along the row, this may just be straight.
    # Tune this based on your placement orientation.
    drive_mm(MOSAIC_CELL_STEP_MM, 70)


def move_one_slot_left():
    print("Move one mosaic slot left")

    # If facing opposite direction on row 2, straight movement can still work.
    drive_mm(MOSAIC_CELL_STEP_MM, 70)


def move_down_to_next_row():
    print("Move down to next mosaic row")

    # This depends heavily on robot orientation.
    # Example: turn, move down, turn back.
    gyro_turn(90)
    drive_mm(MOSAIC_ROW_STEP_MM, 70)


def face_left_for_row_2():
    print("Face left for row 2")
    gyro_turn(180)


def face_right_for_row_3():
    print("Face right for row 3")
    gyro_turn(0)


def place_all_tiles_from_slide():
    """
    Since the slide is FIFO:
    release tile 1 at placement position 1,
    move to next slot,
    release tile 2,
    move to next slot,
    ...
    """

    align_to_first_mosaic_slot()

    for i in range(12):
        target_slot = SERPENTINE_SLOTS[i]

        print("--------------------------------")
        print("Place tile number:", i + 1)
        print("Target real mosaic slot:", target_slot)

        release_one_tile_from_slide()

        if i < 11:
            move_to_next_mosaic_slot(i)

    print("All slide tiles released")


# ============================================================
# 11. FULL PROGRAM
# ============================================================

def solve_mosaic_master():
    # Step 1: Go to mosaic and scan randomized pattern
    go_start_to_mosaic_scan_position()

    mosaic_order = scan_mosaic_serpentine()

    print("Scanned mosaic order:")
    print(mosaic_order)

    # Step 2:
    # Because the slide carrier is FIFO,
    # pickup order must be the SAME as scanned order.
    pickup_order = mosaic_order

    print("Pickup order:")
    print(pickup_order)

    # Step 3: Collect all tiles from source to source
    final_pos = collect_all_tiles_to_slide(pickup_order)

    print("Final pickup position:", final_pos)

    # Step 4: Return from final source to mosaic frame
    go_to_location(final_pos, "mosaic")

    # Step 5: Place tiles one by one by opening the slide door
    place_all_tiles_from_slide()

    print("Mosaic complete")


# ============================================================
# 12. TEST FUNCTIONS
# ============================================================

def test_door_one_tile():
    close_door()
    wait(1000)
    release_one_tile_from_slide()


def test_fixed_pickup_order():
    """
    Test collection without scanning.
    """
    test_order = [
        "yellow", "blue", "green", "white",
        "blue", "yellow", "white", "green",
        "green", "white", "blue", "yellow"
    ]

    go_start_to_mosaic_scan_position()
    final_pos = collect_all_tiles_to_slide(test_order)
    go_to_location(final_pos, "mosaic")
    place_all_tiles_from_slide()


def test_scan_only():
    go_start_to_mosaic_scan_position()
    result = scan_mosaic_serpentine()
    print(result)


# ============================================================
# 13. RUN
# ============================================================

# Test first:
# test_scan_only()
# test_door_one_tile()
# test_fixed_pickup_order()

# Full mission:
solve_mosaic_master()