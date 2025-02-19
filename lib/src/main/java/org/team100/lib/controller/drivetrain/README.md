# Drivetrain

Drivetrain controllers take SwerveState measurement and reference input,
and produce FieldRelativeVelocity commands, which should be fed directly to
SwerveDriveSubsystem.driveInFieldCoords(), or, for references that are known
to be feasible, SwerveDriveSubsystem.driveInFieldCoordsVerbatim().