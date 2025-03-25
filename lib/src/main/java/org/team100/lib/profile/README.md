# profile

Team 100 profiles are stateless incremental reference generators:
you construct a profile with constraints (e.g. on velocity)
and then give the profile a setpoint: it produces the next setpoint.

The SimpleProfile100 interface covers the basic case.

The Profile100 interface adds a few methods related to coordinating
multiple profiles so that they arrive at their goals at the same time,
e.g. for controlling multiple-DOF mechanisms, or the drivetrain.