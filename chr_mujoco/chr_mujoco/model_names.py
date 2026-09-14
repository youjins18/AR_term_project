BASE_JOINT = 'base_free'
TCP_SITE = 'tcp'
TARGET_MARKER_BODY = 'desired_tcp_marker'

ARM_JOINTS = ('J1', 'J2', 'J3')
ROTOR_TILT_JOINTS = tuple(f'rotor_{i}_tilt_joint' for i in range(4))

ACTUATORS = {
    'rotors': tuple(f'rotor_{i}_act' for i in range(4)),
    'rotor_servos': tuple(f'rotor_{i}_tilt_act' for i in range(4)),
    'arm': ('J1_act', 'J2_act', 'J3_act'),
}
