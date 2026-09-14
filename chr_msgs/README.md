# chr_msgs

Shared contracts for the CHR stack. The high-level controller publishes one
`ChrReference`, containing the complete nine-coordinate command
`[x_b, y_b, z_b, roll_b, pitch_b, yaw_b, J1, J2, J3]` (base attitude is encoded as
a quaternion). Low-level controllers consume only their own fields.

Plant inputs are deliberately separate from references:

- `/chr/reference`: desired base pose/twist and arm joint state
- `/chr/actuator/palletrone`: rotor thrust and tilt-servo angles
- `/chr/actuator/arm`: position-loop arm command
- `/chr/state`: simulator or hardware state estimate
