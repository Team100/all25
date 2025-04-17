# semiauto

Team 100 robots do as much as possible automatically.

For example, in 2024, the shooter was almost completely automatic: it would adjust the angle
according to the robot's position on the field, rotate the robot to face the target,
and release the projectile, all with one button.

If desired, the driver could just hold that button down during the entire match,
and the shooter would do its thing whenever it was appropriate.

There were no manual shooter controls.

In 2025, there was a similar level of automation: driving to the correct scoring location,
extending the elevator and wrist to score, were all automatic: there was no manual way to
do any of it.

The examples here illustrate some of the patterns: commands that are aware of their
field-relative positions, and take action appropriately.