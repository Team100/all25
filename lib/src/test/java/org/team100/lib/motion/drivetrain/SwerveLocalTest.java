package org.team100.lib.motion.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.text.CollationElementIterator;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveLocalTest extends Fixtured implements Timeless {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        SwerveModuleCollection modules = fixture.collection;
        SwerveLocal local = fixture.swerveLocal;

        local.setChassisSpeeds(new ChassisSpeeds());
        assertEquals(0, modules.getDesiredStates().frontLeft().speedMetersPerSecond(), 0.001);
        local.defense();
        local.stop();
        local.setRawModuleStates(new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d()))));
        assertEquals(0, local.positions().frontLeft().distanceMeters, 0.001);
    }

    @Test
    void testAligned() {
        SwerveLocal local = fixture.swerveLocal;

        // angle measurements are all at zero
        assertEquals(0, local.positions().frontLeft().angle.get().getRadians(), kDelta);
        assertEquals(0, local.positions().frontRight().angle.get().getRadians(), kDelta);
        assertEquals(0, local.positions().rearLeft().angle.get().getRadians(), kDelta);
        assertEquals(0, local.positions().rearRight().angle.get().getRadians(), kDelta);

        // for motionless setpoint, aligned is meaningless (but returns true).
        // note that many profiles/trajectories start motionless, so the
        // only way to do this is to look ahead a bit.
        assertTrue(local.aligned(new ChassisSpeeds()));

        // proceeding in +x aligns with the wheels
        assertTrue(local.aligned(new ChassisSpeeds(1, 0, 0)));

        // proceeding in +y does not align with the wheels
        assertFalse(local.aligned(new ChassisSpeeds(0, 1, 0)));

        // rotating fast also messes it up
        assertFalse(local.aligned(new ChassisSpeeds(1, 0, 10)));

        // just at the limit
        assertTrue(local.aligned(new ChassisSpeeds(1, 0.05, 0)));
    }

    @Test
    void testAlignedRotationOnly() {
        SwerveLocal local = fixture.swerveLocal;

        assertEquals(0, local.positions().frontLeft().angle.get().getRadians(), kDelta);
        assertEquals(0, local.positions().frontRight().angle.get().getRadians(), kDelta);
        assertEquals(0, local.positions().rearLeft().angle.get().getRadians(), kDelta);
        assertEquals(0, local.positions().rearRight().angle.get().getRadians(), kDelta);

        // pure rotation
        assertFalse(local.aligned(new ChassisSpeeds(0, 0, 1)));
    }

    @Test
    void testSteerAtRest() {
        fixture.collection.reset();
        ChassisSpeeds goal = new ChassisSpeeds(0, 0, 1);
        SwerveLocal local = fixture.swerveLocal;
        local.steerAtRest(goal);
        stepTime();
        assertEquals(-0.013, fixture.collection.positions().frontLeft().angle.get().getRadians(), kDelta);
        assertEquals(0.013, fixture.collection.positions().frontRight().angle.get().getRadians(), kDelta);
        assertEquals(0.013, fixture.collection.positions().rearLeft().angle.get().getRadians(), kDelta);
        assertEquals(-0.013, fixture.collection.positions().rearRight().angle.get().getRadians(), kDelta);
        for (int i = 0; i < 100; ++i) {
            local.steerAtRest(goal);
            stepTime();
        }
        assertEquals(-Math.PI / 4, fixture.collection.positions().frontLeft().angle.get().getRadians(), kDelta);
        assertEquals(Math.PI / 4, fixture.collection.positions().frontRight().angle.get().getRadians(), kDelta);
        assertEquals(Math.PI / 4, fixture.collection.positions().rearLeft().angle.get().getRadians(), kDelta);
        assertEquals(-Math.PI / 4, fixture.collection.positions().rearRight().angle.get().getRadians(), kDelta);
    }
}
