// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2025;

import java.util.ArrayList;
import java.util.List;

import org.team100.frc2025.Swerve.SemiAuto.Hexagon_Nav.LandingDestinationGroup;
import org.team100.frc2025.Swerve.SemiAuto.ReefPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldConstants {

  public enum FieldSector {
    AB(1),
    CD(2),
    EF(3),
    GH(4),
    IJ(5),
    KL(6);

    private final int value; // Field to store the number

    FieldSector(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }

    public static FieldSector fromValue(int value) {
      for (FieldSector sector : FieldSector.values()) {
        if (sector.getValue() == value) {
          return sector;
        }
      }
      throw new IllegalArgumentException("Invalid sector number: " + value);
    }

  }

  public enum ReefDestination {
    LEFT,
    RIGHT,
    CENTER
  }

  public enum ReefAproach {
    CCW,
    CW
  }

  public enum CoralStation {
    Left,
    Right
  }

  public static FieldConstants.FieldSector getSector(Pose2d pose) {
    Translation2d target = FieldConstants.getReefCenter().minus(pose.getTranslation());
    Rotation2d targetAngle = target.getAngle();
    double angle = targetAngle.getDegrees();

    if (angle >= -30 && angle <= 30) {
      return FieldSector.AB;
    } else if (angle >= -90 && angle < -30) {
      return FieldSector.KL;
    } else if (angle >= -150 && angle <= -90) {
      return FieldSector.IJ;
    } else if (angle >= 150 || angle <= -150) {
      return FieldSector.GH;
    } else if (angle >= 90 && angle <= 150) {
      return FieldSector.EF;
    } else if (angle >= 30 && angle <= 90) {
      return FieldSector.CD;
    } else {
      return FieldSector.AB;
    }
  }

  public static Rotation2d getSectorAngle(FieldSector sector) {
    switch (sector) {
      case AB:
        return Rotation2d.fromDegrees(180);
      case CD:
        return Rotation2d.fromDegrees(-120);
      case EF:
        return Rotation2d.fromDegrees(-60);
      case GH:
        return Rotation2d.fromDegrees(0);
      case IJ:
        return Rotation2d.fromDegrees(60);
      case KL:
        return Rotation2d.fromDegrees(120);
      default:
        return Rotation2d.fromDegrees(0);
    }
  }

  public static Rotation2d getLandingAngle(FieldSector sector, ReefAproach approach) {
    Rotation2d rotation = new Rotation2d(-90);

    switch (sector) {
      case AB:
        rotation = Rotation2d.fromDegrees(-90);
        break;
      case CD:
        rotation = Rotation2d.fromDegrees(-30);
        break;
      case EF:
        rotation = Rotation2d.fromDegrees(30);
        break;
      case GH:
        rotation = Rotation2d.fromDegrees(90);
        break;
      case IJ:
        rotation = Rotation2d.fromDegrees(150);
        break;
      case KL:
        rotation = Rotation2d.fromDegrees(210);
        break;
      default:
        rotation = Rotation2d.fromDegrees(0);
        break;
    }

    switch (approach) {
      case CCW:
        return rotation;
      case CW:
        return rotation.rotateBy(Rotation2d.fromDegrees(180));
      default:
        return rotation;
    }

  }

  public static Rotation2d calculateAnchorPointDelta(Rotation2d originalRotation, ReefAproach approach) {
    switch (approach) {
      case CCW:
        return originalRotation.plus(Rotation2d.fromDegrees(30));
      case CW:
        return originalRotation.minus(Rotation2d.fromDegrees(30));
      default:
        return originalRotation;
    }
  }

  public static double getDistanceToReefCenter(Pose2d pose) {
    Translation2d target = FieldConstants.getReefCenter().minus(pose.getTranslation());
    return target.getNorm();
  }

  public static double getDistanceToReefCenter(Translation2d translation) {
    Translation2d target = FieldConstants.getReefCenter().minus(translation);
    return target.getNorm();
  }

  public static Rotation2d angleToReefCenter(Translation2d t) {
    Translation2d target = FieldConstants.getReefCenter().minus(t);
    Rotation2d targetAngle = target.getAngle();
    return targetAngle;
  }

  public static Translation2d getReefCenter() { // blue
    return new Translation2d(4.489, 4.026);
  }

  public static double getOrbitWaypointRadius() {
    return 2.1; // 2.4
  }

  public static double getOrbitDestinationRadius() {
    return 1.7;
  }

  public static double getReefOffset() {
    return 0.1645; // 0.1524
  }

  public static Translation2d getOrbitWaypoint(Rotation2d angle) {
    Translation2d reefCenter = getReefCenter();
    double x = reefCenter.getX() + getOrbitWaypointRadius() * angle.getCos();
    double y = reefCenter.getY() + getOrbitWaypointRadius() * angle.getSin();

    return new Translation2d(x, y);

  }

  public static Translation2d getOrbitWaypoint(Rotation2d angle, double radius) {
    Translation2d reefCenter = getReefCenter();
    double x = reefCenter.getX() + (radius * angle.getCos());
    double y = reefCenter.getY() + (radius * angle.getSin());

    return new Translation2d(x, y);

  }

  public static Rotation2d calculateDeltaSpline(Rotation2d originalRotation, Rotation2d deltaRotation, ReefAproach approach, double scale) {
    
    Translation2d originalTranslation = new Translation2d(1, originalRotation);
    Translation2d deltaTranslation = new Translation2d(scale, deltaRotation);
    Translation2d newTranslation = originalTranslation.plus(deltaTranslation);

    return newTranslation.getAngle();
  }


  public static LandingDestinationGroup getRotationGroup(ReefAproach approach, FieldSector end, double kScaleFactor) {
    Translation2d parallelLandingVector = new Translation2d(1, FieldConstants.getLandingAngle(end, approach));
    Translation2d parallelLandingVectorUnit = new Translation2d(kScaleFactor,
        FieldConstants.getLandingAngle(end, approach));

    Translation2d perpendicularLandingVector;
    Translation2d perpendicularDestinationVector;

    if (approach == ReefAproach.CCW) {
      perpendicularLandingVector = parallelLandingVectorUnit.rotateBy(Rotation2d.fromDegrees(-90));
      perpendicularDestinationVector = parallelLandingVectorUnit.rotateBy(Rotation2d.fromDegrees(90));
    } else {
      perpendicularLandingVector = parallelLandingVectorUnit.rotateBy(Rotation2d.fromDegrees(90));
      perpendicularDestinationVector = parallelLandingVectorUnit.rotateBy(Rotation2d.fromDegrees(-90));
    }

    Translation2d landingVector = parallelLandingVector.plus(perpendicularLandingVector);
    Translation2d destVec = parallelLandingVector.plus(perpendicularDestinationVector);

    return new LandingDestinationGroup(landingVector.getAngle(), destVec.getAngle());
  }

  public static Translation2d getOrbitLandingZone(FieldSector destinationSector, ReefAproach approach) {
    Translation2d reefCenter = getReefCenter();
    Rotation2d sectorAngle = getSectorAngle(destinationSector);

    double x = reefCenter.getX() + getOrbitDestinationRadius() * sectorAngle.getCos();
    double y = reefCenter.getY() + getOrbitDestinationRadius() * sectorAngle.getSin();

    double dx = 0;
    double dy = 0;

    Rotation2d newRotation = Rotation2d.fromDegrees(sectorAngle.getDegrees() + 90);
    dy = (1.0 * Math.sin(newRotation.getRadians()));
    dx = (1.0 * Math.cos(newRotation.getRadians()));

    switch (approach) {
      case CW:
        return new Translation2d(x += dx, y += dy);
      case CCW:
        return new Translation2d(x -= dx, y -= dy);

    }

    return new Translation2d(x, y);

  }

  public static Translation2d getOrbitDestination(FieldSector destinationSector, ReefDestination destinationPoint) {
    Translation2d reefCenter = getReefCenter();
    Rotation2d sectorAngle = getSectorAngle(destinationSector);

    double x = reefCenter.getX() + getOrbitDestinationRadius() * sectorAngle.getCos();
    double y = reefCenter.getY() + getOrbitDestinationRadius() * sectorAngle.getSin();
    double dx = 0;
    double dy = 0;

    Rotation2d newRotation = Rotation2d.fromDegrees(sectorAngle.getDegrees() + 90);
    dy = (getReefOffset() * Math.sin(newRotation.getRadians()));
    dx = (getReefOffset() * Math.cos(newRotation.getRadians()));

    switch (destinationPoint) {
      case RIGHT:
        return new Translation2d(x += dx, y += dy);
      case LEFT:
        return new Translation2d(x -= dx, y -= dy);
      case CENTER:
        return new Translation2d(x, y);

    }

    return new Translation2d(x, y);

  }

  public static Translation2d getOrbitDestination(FieldSector destinationSector, ReefDestination destinationPoint, double radius) {
    Translation2d reefCenter = getReefCenter();
    Rotation2d sectorAngle = getSectorAngle(destinationSector);

    double x = reefCenter.getX() + radius * sectorAngle.getCos();
    double y = reefCenter.getY() + radius * sectorAngle.getSin();

    double dx = 0;
    double dy = 0;

    Rotation2d newRotation = Rotation2d.fromDegrees(sectorAngle.getDegrees() + 90);
    dy = (getReefOffset() * Math.sin(newRotation.getRadians()));
    dx = (getReefOffset() * Math.cos(newRotation.getRadians()));

    switch (destinationPoint) {
      case RIGHT:
        return new Translation2d(x += dx, y += dy);
      case LEFT:
        return new Translation2d(x -= dx, y -= dy);
      case CENTER:
        return new Translation2d(x, y);

    }

    return new Translation2d(x, y);

  }

  public static ReefPath findShortestPath(int start, int target) {
    List<Integer> pathClockwise = new ArrayList<>();
    List<Integer> pathCounterClockwise = new ArrayList<>();

    // Clockwise path
    int side = start;
    while (side != target) {
      pathCounterClockwise.add(side);
      side = (side % 6) + 1; // Move clockwise
    }
    pathCounterClockwise.add(target); // Add target side

    // Counterclockwise path
    side = start;
    while (side != target) {
      pathClockwise.add(side);
      side = (side - 2 + 6) % 6 + 1; // Move counterclockwise
    }
    pathClockwise.add(target); // Add target side

    // Return the shorter path

    if (pathClockwise.size() < pathCounterClockwise.size()) {
      return new ReefPath(pathClockwise, ReefAproach.CW);
    } else {
      return new ReefPath(pathCounterClockwise, ReefAproach.CCW);
    }
  }
}
