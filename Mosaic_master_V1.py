from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait


# ============================================================
# HARDWARE SETUP
# ============================================================

hub = PrimeHub()

left_motor  = Motor(Port.A)
right_motor = Motor(Port.B)

grab_motor           = Motor(Port.C)
scanner_slider_motor = Motor(Port.D)
door_motor           = Motor(Port.E)

color_sensor_A = ColorSensor(Port.F)

robot = DriveBase(
    left_motor,
    right_motor,
    wheel_diameter=56,
    axle_track=120
)


# ============================================================
# BASIC TUNING
# ============================================================

BLACK_THRESHOLD = 35

DRIVE_SPEED  = 200
SLOW_SPEED   = 100

TURN_SPEED   = 160
TURN_KP      = 3
TURN_TOLERANCE = 2

# Nudge distance (mm) forward after detecting a black line
# to avoid re-detecting the same line on the next go_until_black()
LINE_PASS_NUDGE = 60

# Mosaic cell dimensions (mm) — tune on actual mat
CELL_WIDTH  = 55   # horizontal distance between cell centers
ROW_HEIGHT  = 55   # vertical distance between row centers


# ============================================================
# COLOR MAP
#
# Maps sensor Color reading to source label (A / B / C / D).
# Tune these to match the actual tile colors in your competition set.
#
# WRO 2026 mosaic tile colors: yellow, blue, green, white
# ============================================================

COLOR_MAP = {
    Color.YELLOW : "A",
    Color.BLUE   : "B",
    Color.GREEN  : "C",
    Color.WHITE  : "D",
}


# ============================================================
# SOURCE LAYOUT
# TOP TO BOTTOM:
#
# A  (yellow)  level 0
# B  (blue)    level 1
# C  (green)   level 2
# D  (white)   level 3
# ============================================================

SOURCE_LEVEL = {
    "A": 0,
    "B": 1,
    "C": 2,
    "D": 3
}


# ============================================================
# BASIC MOVEMENT FUNCTIONS
# ============================================================

def is_black():
    return color_sensor_A.reflection() < BLACK_THRESHOLD


def stop_robot():
    robot.stop()
    wait(100)


def drive_mm(distance, speed=DRIVE_SPEED):
    robot.settings(straight_speed=abs(speed))
    robot.straight(distance)
    wait(100)


def go_until_black(speed=SLOW_SPEED):
    robot.drive(speed, 0)
    while not is_black():
        wait(10)
    stop_robot()


def set_gyro_zero():
    hub.imu.reset_heading(0)
    wait(200)


def turn_pivot(target_angle):
    """
    Absolute gyro pivot turn.
      0   = north (forward / into sources)
      90  = east  (right / downward along source lane)
      180 = south (backward / toward main line)
     -90  = west  (left  / upward along source lane)
    """
    while True:
        current = hub.imu.heading()
        error   = target_angle - current

        if abs(error) <= TURN_TOLERANCE:
            break

        turn_rate = error * TURN_KP
        turn_rate = max(-TURN_SPEED, min(TURN_SPEED, turn_rate))

        robot.drive(0, turn_rate)
        wait(10)

    stop_robot()


def turn_yaw(angle):
    """
    Relative correction turn from current heading.
    Positive = clockwise (right), Negative = counter-clockwise (left).
    """
    target = hub.imu.heading() + angle
    turn_pivot(target)


# ============================================================
# GO TO MOSAIC
# ============================================================

def go_to_mosaic():
    """
    Navigate from start position to cell [0] of the mosaic frame.

    Steps:
    1. Zero gyro at start.
    2. Turn 50 deg — aligns exit angle from start area
       toward the black line that leads to the mosaic lane.
    3. Drive until color sensor A hits that black line.
    4. Turn 90 deg to face the mosaic lane.
    5. Re-zero gyro — this becomes 0 deg for the rest of the run.
    6. Small yaw correction + drive to mosaic entry black line.
    7. Straighten (turn_pivot 0) and drive to first mosaic cell.
    """

    print("Going to mosaic")

    set_gyro_zero()

    # 50 deg: aligns with the angled exit path from the start area
    turn_pivot(50)
    go_until_black()

    turn_pivot(90)
    set_gyro_zero()   # re-zero: 0 deg now = facing into mosaic / sources

    turn_yaw(15)
    go_until_black()

    turn_pivot(0)
    go_until_black()

    print("Arrived at mosaic cell [0] / ready to scan")


# ============================================================
# SCAN MOSAIC
# ============================================================

def read_color_at_cell():
    """
    Read color sensor at current cell and return source label A/B/C/D.
    Brief pause before reading to let sensor settle.
    Falls back to "A" and prints a warning if color is unrecognised.
    """
    wait(150)
    detected = color_sensor_A.color()
    label    = COLOR_MAP.get(detected, None)

    if label is None:
        print("WARNING: Unrecognised color:", detected, "- defaulting to A")
        label = "A"
    else:
        print("  Cell color:", detected, "->", label)

    return label


def run_scanner_slider():
    """Extend scanner slider arm over the mosaic frame."""
    print("Scanner slider run")
    scanner_slider_motor.run_angle(300, 720, then=Stop.HOLD)
    wait(200)


def scan_mosaic():
    """
    Scan all 12 mosaic cells in serpentine order. Returns mosaic_list.

    Robot starts at cell [0] (top-left) facing 0 deg.

    Serpentine scan order:
      Row 1 (left  -> right): [0]  [1]  [2]  [3]
      Row 2 (right -> left):  [4]  [5]  [6]  [7]
      Row 3 (left  -> right): [8]  [9]  [10] [11]

    After scanning, robot repositions back to cell [0] so that
    go_mosaic_to_source() can navigate from a known reference point.
    """

    print("Scanning mosaic...")
    mosaic_list = []

    run_scanner_slider()

    # ---- Row 1: left to right (heading 0) ----
    for i in range(4):
        color = read_color_at_cell()
        mosaic_list.append(color)
        print("  Row 1 cell", i, "=", color)
        if i < 3:
            drive_mm(CELL_WIDTH, SLOW_SPEED)
    # Robot now at cell [3], heading 0

    # ---- Step down to Row 2 ----
    # Turn right (+90), drive one row height, turn to face right-to-left (180)
    turn_pivot(90)
    drive_mm(ROW_HEIGHT, SLOW_SPEED)
    turn_pivot(180)
    # Robot now at cell [4] (start of row 2), heading 180

    # ---- Row 2: right to left (heading 180) ----
    for i in range(4):
        color = read_color_at_cell()
        mosaic_list.append(color)
        print("  Row 2 cell", i, "=", color)
        if i < 3:
            drive_mm(CELL_WIDTH, SLOW_SPEED)
    # Robot now at cell [7], heading 180

    # ---- Step down to Row 3 ----
    # Turn right from 180 = pivot to 90, drive one row height, face left-to-right (0)
    turn_pivot(90)
    drive_mm(ROW_HEIGHT, SLOW_SPEED)
    turn_pivot(0)
    # Robot now at cell [8] (start of row 3), heading 0

    # ---- Row 3: left to right (heading 0) ----
    for i in range(4):
        color = read_color_at_cell()
        mosaic_list.append(color)
        print("  Row 3 cell", i, "=", color)
        if i < 3:
            drive_mm(CELL_WIDTH, SLOW_SPEED)
    # Robot now at cell [11], heading 0

    # ---- Return to cell [0] (mosaic entry / reference point) ----
    # Reverse row 3: turn 180, drive back 3 cells to [8]
    turn_pivot(180)
    drive_mm(3 * CELL_WIDTH, SLOW_SPEED)

    # Step back up to row 2 start: turn left (-90 from 180 = pivot to -90),
    # drive one row height, arrive at [7]
    turn_pivot(-90)
    drive_mm(ROW_HEIGHT, SLOW_SPEED)

    # Step back up to row 1 start: same direction, drive one row height
    drive_mm(ROW_HEIGHT, SLOW_SPEED)

    # Straighten to face 0 (into sources / north)
    turn_pivot(0)
    # Robot is now back at cell [0], heading 0

    print("Scan complete. mosaic_list =", mosaic_list)

    return mosaic_list


# ============================================================
# GRAB / STORE FUNCTIONS
# ============================================================

def grab_and_lift():
    print("Grab and lift block")
    grab_motor.run_target(400, 80,  then=Stop.HOLD)
    wait(150)
    grab_motor.run_target(400, 150, then=Stop.HOLD)
    wait(150)


def store_to_slide():
    """Release block into the FIFO slide carrier."""
    print("Store block into FIFO slide carrier")
    grab_motor.run_target(400, 30, then=Stop.HOLD)
    wait(150)
    grab_motor.run_target(400, 0,  then=Stop.HOLD)
    wait(150)


def first_block():
    """
    Collect the FIRST block when entering a new source.

    - Drive until color sensor A sees the pickup position black marker
      (this marker is inside the source lane, past the intersection).
    - Turn 15 deg to offset grabber onto the tile.
    - Grab, lift and store.

    IMPORTANT: Verify a black marker exists at the pickup position
    in every source lane on the actual mat. If not, replace
    go_until_black() with drive_mm(pickup_distance) tuned per source.
    """
    print("Take FIRST block from this source")

    go_until_black()
    stop_robot()

    turn_pivot(15)  # 15 deg offset aligns grabber with tile

    grab_and_lift()
    store_to_slide()


def second_block():
    """
    Collect the NEXT block from the SAME source (robot stays put).

    - Turn -15 deg to offset to the second tile position.
    - Grab, lift and store.
    """
    print("Take NEXT block from same source")

    turn_pivot(-15)  # opposite offset for second tile

    grab_and_lift()
    store_to_slide()


# ============================================================
# RETURN TO MAIN LINE
# ============================================================

def return_to_main_line():
    """
    Leave current source and return to the main navigation line.
    Called every time the robot needs to change source.

    - Turn 180 deg (face back toward main line).
    - Drive until color sensor A sees the main line black marker.
    - Stop.
    """
    print("Return to main line")
    turn_pivot(180)
    go_until_black()
    stop_robot()


# ============================================================
# DIRECTION DECISION
# ============================================================

def decide_turn_direction(current_location, next_color):
    """
    Determine whether to travel up (LEFT) or down (RIGHT) the source lane.

    A (level 0) is top, D (level 3) is bottom.
    Smaller level = higher up = turn LEFT (-90).
    Larger level  = lower down = turn RIGHT (90).
    """
    current_level = SOURCE_LEVEL[current_location]
    target_level  = SOURCE_LEVEL[next_color]

    if target_level < current_level:
        return "LEFT"
    elif target_level > current_level:
        return "RIGHT"
    else:
        return "SAME"


def count_lines_to_target(current_location, next_color):
    """
    Number of black line detections to count when traveling
    from current source to target source along the source lane.
    """
    current_level = SOURCE_LEVEL[current_location]
    target_level  = SOURCE_LEVEL[next_color]
    return abs(target_level - current_level)


# ============================================================
# TRAVEL BETWEEN SOURCES
# ============================================================

def go_until_nth_black(target_count):
    """
    Drive forward counting black line detections until target_count is reached.

    After each detection except the last, nudge forward LINE_PASS_NUDGE mm
    to clear the current black line before resuming, preventing re-detection.
    """
    count = 0
    while count < target_count:
        go_until_black()
        count += 1
        print("  Black detected:", count, "/", target_count)
        if count < target_count:
            drive_mm(LINE_PASS_NUDGE, SLOW_SPEED)
    stop_robot()


def go_source_to_source(current_location, next_color):
    """
    Move robot from current_location source to next_color source.

    Note: The same-source (second_block) case is already handled in
    collect_all_blocks() before this function is called.
    The check below is a safety fallback only and should not trigger
    in normal operation.
    """
    print("Move from", current_location, "to", next_color)

    # Safety fallback only
    if current_location == next_color:
        second_block()
        return

    return_to_main_line()

    direction  = decide_turn_direction(current_location, next_color)
    line_count = count_lines_to_target(current_location, next_color)

    if direction == "LEFT":
        print("  Target is above current source -> turn LEFT (-90)")
        turn_pivot(-90)
    elif direction == "RIGHT":
        print("  Target is below current source -> turn RIGHT (90)")
        turn_pivot(90)

    go_until_nth_black(line_count)

    # Enter target source facing inward
    turn_pivot(0)

    first_block()


# ============================================================
# MOSAIC TO FIRST SOURCE
# ============================================================

def go_mosaic_to_source(next_color):
    """
    First movement after scanning. Robot starts at mosaic cell [0], heading 0.

    Layout reference (sources relative to mosaic):
      A = 2 intersections to the RIGHT (turn 90)
      B = 1 intersection  to the RIGHT (turn 90)
      C = 1 intersection  to the LEFT  (turn -90)
      D = 2 intersections to the LEFT  (turn -90)

    Procedure:
    - Turn toward the correct side.
    - Count the correct number of black line intersections.
    - Face inward (turn_pivot 0) to enter source.
    - Collect first block.
    """
    print("Mosaic to source", next_color)

    if next_color == "A":
        turn_pivot(90)
        go_until_black()
        drive_mm(LINE_PASS_NUDGE, SLOW_SPEED)
        go_until_black()
        turn_pivot(0)

    elif next_color == "B":
        turn_pivot(90)
        go_until_black()
        turn_pivot(0)

    elif next_color == "C":
        turn_pivot(-90)
        go_until_black()
        turn_pivot(0)

    elif next_color == "D":
        turn_pivot(-90)
        go_until_black()
        drive_mm(LINE_PASS_NUDGE, SLOW_SPEED)
        go_until_black()
        turn_pivot(0)

    first_block()


# ============================================================
# COLLECT ALL 12 BLOCKS
# ============================================================

def collect_all_blocks(mosaic_list):
    """
    Main collection loop. Iterates mosaic_list index 0 to 11.
    Stores every block into the FIFO slide carrier in scan order.
    Returns current_location after all 12 pickups.

    Decision logic per index:
      - Robot at mosaic          -> go_mosaic_to_source()
      - Robot at correct source  -> second_block()  (stay, no travel)
      - Robot at wrong source    -> go_source_to_source()
    """
    current_location = "mosaic"

    for index in range(12):
        next_color = mosaic_list[index]

        print("------------------------------------")
        print("Index:", index)
        print("Current location:", current_location)
        print("Next color needed:", next_color)

        if current_location == "mosaic":
            go_mosaic_to_source(next_color)

        elif current_location == next_color:
            second_block()

        else:
            go_source_to_source(current_location, next_color)

        current_location = next_color

        print("Updated current_location =", current_location)

    return current_location


# ============================================================
# RETURN FROM FINAL SOURCE TO MOSAIC
# ============================================================

def return_source_to_mosaic(current_location):
    """
    After all 12 blocks are collected, navigate from the final source
    back to mosaic cell [0] for placement.

    This is the mirror of go_mosaic_to_source():
      A (2 right of mosaic): return to main, turn LEFT (-90), pass 2 lines
      B (1 right of mosaic): return to main, turn LEFT (-90), pass 1 line
      C (1 left  of mosaic): return to main, turn RIGHT (90), pass 1 line
      D (2 left  of mosaic): return to main, turn RIGHT (90), pass 2 lines

    After counting lines, turn_pivot(0) faces the mosaic.
    Final go_until_black() aligns to mosaic cell [0] entry marker.
    """
    print("Return from source", current_location, "to mosaic")

    return_to_main_line()

    if current_location == "A":
        turn_pivot(-90)
        go_until_black()
        drive_mm(LINE_PASS_NUDGE, SLOW_SPEED)
        go_until_black()

    elif current_location == "B":
        turn_pivot(-90)
        go_until_black()

    elif current_location == "C":
        turn_pivot(90)
        go_until_black()

    elif current_location == "D":
        turn_pivot(90)
        go_until_black()
        drive_mm(LINE_PASS_NUDGE, SLOW_SPEED)
        go_until_black()

    # Face the mosaic frame
    turn_pivot(0)

    # Align to cell [0] entry marker
    go_until_black()

    print("Arrived at mosaic / ready to place")


# ============================================================
# PLACE TILES FROM FIFO SLIDE
# ============================================================

def release_one_tile():
    """Open door briefly to release one tile from the FIFO slide."""
    print("Release one tile from FIFO slide")
    door_motor.run_target(300, 60, then=Stop.HOLD)
    wait(250)
    door_motor.run_target(300, 0, then=Stop.HOLD)
    wait(150)


def move_to_next_mosaic_slot(i):
    """
    Move to the next mosaic cell after placing tile at index i.

    Placement serpentine (mirrors scan order):
      Row 1 [0->3]:  heading 0   (left  -> right)
      Row 2 [4->7]:  heading 180 (right -> left)
      Row 3 [8->11]: heading 0   (left  -> right)

    Row transitions turn right (90), drive one ROW_HEIGHT, then
    turn to face the new row direction.

    CELL_WIDTH and ROW_HEIGHT need physical tuning on the actual mat.
    """

    # Row 1: advance to next cell (heading 0)
    if i in [0, 1, 2]:
        drive_mm(CELL_WIDTH, SLOW_SPEED)

    # End of Row 1 -> step down to Row 2, face right-to-left (180)
    elif i == 3:
        turn_pivot(90)
        drive_mm(ROW_HEIGHT, SLOW_SPEED)
        turn_pivot(180)

    # Row 2: advance to next cell (heading 180 = right-to-left)
    elif i in [4, 5, 6]:
        drive_mm(CELL_WIDTH, SLOW_SPEED)

    # End of Row 2 -> step down to Row 3, face left-to-right (0)
    elif i == 7:
        turn_pivot(90)
        drive_mm(ROW_HEIGHT, SLOW_SPEED)
        turn_pivot(0)

    # Row 3: advance to next cell (heading 0)
    elif i in [8, 9, 10]:
        drive_mm(CELL_WIDTH, SLOW_SPEED)

    # i == 11 is the last tile — no movement after final release


def place_tiles_serpentine():
    """
    Release all 12 tiles one by one in serpentine placement order.

    Because the FIFO slide releases tiles in the same order they were
    collected, and collection order matches scan order, no color
    decisions are needed here. Just release and move.
    """
    print("Start placing tiles serpentine")

    for i in range(12):
        print("Place tile index", i)
        release_one_tile()
        if i < 11:
            move_to_next_mosaic_slot(i)

    print("All tiles placed")


# ============================================================
# MAIN PROGRAM
# ============================================================

def main():
    # 1. Navigate from start to mosaic cell [0]
    go_to_mosaic()

    # 2. Scan all 12 cells in serpentine order
    #    Returns to cell [0] after scanning
    mosaic_list = scan_mosaic()

    print("Scanned mosaic list:")
    print(mosaic_list)

    # 3. Collect all 12 blocks from sources in scan order
    final_location = collect_all_blocks(mosaic_list)

    print("Finished collecting all blocks.")
    print("Final source location:", final_location)

    # 4. Return to mosaic frame for placement
    return_source_to_mosaic(final_location)

    # 5. Release tiles in serpentine placement order
    place_tiles_serpentine()

    print("Mission finished")


main()