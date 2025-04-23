package org.team100.lib.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public record LandingDestinationGroup(Rotation2d landingSpline, Rotation2d destinationSpline) {
}
