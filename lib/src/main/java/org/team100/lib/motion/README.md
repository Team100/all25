# motion

A Team 100 simple motion control consists of several components:

- a _motor_:  may support closed-loop positional or velocity control, or not.
- a _sensor_: usually the Team 100 magnetic absolute rotary sensor.
- a _mechanism_: combines the motor and sensor, and includes the gear ratio. 
- a _profile_: plans motion to limit velocity or acceleration
- _feedback_: tries to keep the mechanism on the profile
- a _servo_: a container for all of the above
- a _subsystem_: a configuration for all of the above, used by the scheduler
- some set of _commands_ that manipulates the subsystem

For examples of assembling these components into subsytems, and how to control them, look at lib/examples/motion.