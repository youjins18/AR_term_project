# arm_controller

This node is intentionally a pass-through boundary. It selects J1--J3 from the
whole-body `/chr/reference`, clamps them to the model limits, attaches
position-loop gains, and publishes `/chr/actuator/arm`.

The MuJoCo plant implements the actual PD torque law, mirroring a Dynamixel-style
position-control boundary. MuJoCo bias-torque feedforward is enabled in the
simulation configuration to remove gravity-induced steady-state error; disable it
before connecting a hardware driver until signs and units are verified. A
stale-reference watchdog holds the measured arm position.
