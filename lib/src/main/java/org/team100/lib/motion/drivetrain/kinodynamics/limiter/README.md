# limiter

The limiter package is concerned with making the drivetrain inputs feasible.

The entry point is SwerveLimiter, which applies the rest of the classes here in a
chain.

The idea is distantly derived from 254's SwerveSetpointGenerator.  The most notable
difference is that the setpoint generator operated in robot-relative coordinates
(I think because 254's code had evolved from tank drive).  Using robot-relative
coordinates made veering correction difficult -- the "limits" would tend to mix
rotational and translational inputs, making veering worse.

The Team 100 SwerveLimiter operates in field-relative coordinates, and so doesn't
have the veering problem.

The limiters here are concerned with velocity and acceleration.  In the fall of 2024,
I tried limiting jerk as well, but my simple attempts resulted in lag and overshoot.
See studies/high_order_limiters.