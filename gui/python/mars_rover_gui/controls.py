GAS = 1 << 0
BRAKE = 1 << 1
REVERSE = 1 << 2
CLUTCH = 1 << 3
TILT_LEFT = 1 << 4
TILT_RIGHT = 1 << 5
GEAR_UP = 1 << 6
GEAR_DOWN = 1 << 7
DRIVE = 1 << 8
IGNITION = 1 << 9
CHARGE = 1 << 10
LIDAR = 1 << 11
HEATER = 1 << 12
JUMP = 1 << 13
CLIMB = 1 << 14
PROPELLER = 1 << 15
JUMP_FRONT = 1 << 17
JUMP_REAR = 1 << 18
PISTON_FRONT = 1 << 19
PISTON_REAR = 1 << 20
BALLAST_BLOW = 1 << 21
BALLAST_FLOOD = 1 << 22
THRUSTER = 1 << 23

HELD_CONTROLS = (
    ("gas", "D / →\nGAS", GAS),
    ("reverse", "A / ←\nREVERSE", GAS | REVERSE),
    ("brake", "S / ↓\nBRAKE", BRAKE),
    ("clutch", "C / SHIFT\nCLUTCH", CLUTCH),
    ("tilt_left", "J\nTILT LEFT", TILT_LEFT),
    ("tilt_right", "L\nTILT RIGHT", TILT_RIGHT),
    ("heater", "H\nENGINE HEAT", HEATER),
    ("jump", "K\nJUMP BOTH", JUMP),
    ("jump_front", "CTRL+K\nJUMP FRONT", JUMP_FRONT),
    ("jump_rear", "ALT+K\nJUMP REAR", JUMP_REAR),
    ("piston", "I\nPUSH BOTH", PISTON_FRONT | PISTON_REAR),
    ("piston_front", "CTRL+I\nPUSH FRONT", PISTON_FRONT),
    ("piston_rear", "ALT+I\nPUSH REAR", PISTON_REAR),
    ("blow", "Y\nBALLAST BLOW", BALLAST_BLOW),
    ("flood", "U\nBALLAST FLOOD", BALLAST_FLOOD),
    ("thruster", "SPACE\nROCKET", THRUSTER),
)

PULSE_CONTROLS = (
    ("gear_down", "Z\nGEAR DOWN", GEAR_DOWN),
    ("gear_up", "X\nGEAR UP", GEAR_UP),
    ("drive", "V\nRWD/FWD/AWD", DRIVE),
    ("ignition", "E\nIGNITION", IGNITION),
    ("charge", "F\nSOLAR PANEL", CHARGE),
    ("lidar", "G\nLIDAR", LIDAR),
    ("climb", "B\nCLIMB MODE", CLIMB),
    ("propeller", "P\nPROPELLER", PROPELLER),
)

PULSE_KEYS = {
    "z": GEAR_DOWN,
    "x": GEAR_UP,
    "v": DRIVE,
    "e": IGNITION,
    "f": CHARGE,
    "g": LIDAR,
    "b": CLIMB,
    "p": PROPELLER,
}


def keyboard_action(keys: set[str]) -> int:
    action = 0
    if {"d", "right"} & keys:
        action |= GAS
    if {"a", "left"} & keys:
        action |= GAS | REVERSE
    if {"s", "down"} & keys:
        action |= BRAKE
    if {"c", "shift_l", "shift_r"} & keys:
        action |= CLUTCH
    if "j" in keys:
        action |= TILT_LEFT
    if "l" in keys:
        action |= TILT_RIGHT
    if "h" in keys:
        action |= HEATER
    ctrl = bool({"control_l", "control_r"} & keys)
    alt = bool({"alt_l", "alt_r"} & keys)
    if "k" in keys:
        action |= JUMP_FRONT if ctrl else JUMP_REAR if alt else JUMP
    if "i" in keys:
        action |= PISTON_FRONT if ctrl else PISTON_REAR if alt else PISTON_FRONT | PISTON_REAR
    if "y" in keys:
        action |= BALLAST_BLOW
    if "u" in keys:
        action |= BALLAST_FLOOD
    if "space" in keys:
        action |= THRUSTER
    return action
