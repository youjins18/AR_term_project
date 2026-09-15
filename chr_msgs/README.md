# chr_msgs

Shared contracts for the CHR stack. The high-level controller publishes one
`ChrReference`, containing the full base pose and J1--J3 command. Base attitude is
encoded as a quaternion, but the current CHR invariant fixes roll and pitch to
zero. The independent command is therefore
`[x_b, y_b, z_b, yaw_b, J1, J2, J3]`. Controllers consume only their own fields.

Plant inputs are deliberately separate from references:

- `/chr/reference`: desired base pose/twist and arm joint state
- `/chr/actuator/palletrone`: rotor thrust and tilt-servo angles
- `/chr/actuator/arm`: position-loop arm command
- `/chr/state`: simulator or hardware state estimate
