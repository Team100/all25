package org.team100.frc2024.Swerve.SemiAuto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public record PoseSet(List<Pose2d> poses, List<Rotation2d> headings) {
}
