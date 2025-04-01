package org.team100.frc2025;

import org.junit.jupiter.api.Test;
import org.team100.lib.commands.drivetrain.FieldConstants;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;

import edu.wpi.first.math.geometry.Translation2d;

public class SetpointsTest {
    @Test
    void testSetA(){
        Translation2d t = FieldConstants.getOrbitDestination(FieldSector.AB, ReefDestination.LEFT, 1.4);
        Translation2d t1 = FieldConstants.getOrbitDestination(FieldSector.AB, ReefDestination.RIGHT, 1.4);
        Translation2d t2 = FieldConstants.getOrbitDestination(FieldSector.CD, ReefDestination.LEFT, 1.4);
        Translation2d t3 = FieldConstants.getOrbitDestination(FieldSector.CD, ReefDestination.RIGHT, 1.4);
        Translation2d t4 = FieldConstants.getOrbitDestination(FieldSector.EF, ReefDestination.LEFT, 1.4);
        Translation2d t5 = FieldConstants.getOrbitDestination(FieldSector.EF, ReefDestination.RIGHT, 1.4);
        Translation2d t6 = FieldConstants.getOrbitDestination(FieldSector.GH, ReefDestination.LEFT, 1.4);
        Translation2d t7 = FieldConstants.getOrbitDestination(FieldSector.GH, ReefDestination.RIGHT, 1.4);
        Translation2d t8 = FieldConstants.getOrbitDestination(FieldSector.IJ, ReefDestination.LEFT, 1.4);
        Translation2d t9 = FieldConstants.getOrbitDestination(FieldSector.IJ, ReefDestination.RIGHT, 1.4);
        Translation2d t10 = FieldConstants.getOrbitDestination(FieldSector.KL, ReefDestination.LEFT, 1.4);
        Translation2d t11 = FieldConstants.getOrbitDestination(FieldSector.KL, ReefDestination.RIGHT, 1.4);
        System.out.println(t);
        System.out.println(t1);
        System.out.println(t2);
        System.out.println(t3);
        System.out.println(t4);
        System.out.println(t5);
        System.out.println(t6);
        System.out.println(t7);
        System.out.println(t8);
        System.out.println(t9);
        System.out.println(t10);
        System.out.println(t11);

    }

}
