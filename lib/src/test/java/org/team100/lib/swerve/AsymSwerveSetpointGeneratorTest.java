package org.team100.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleStates;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class AsymSwerveSetpointGeneratorTest {
    private static final double kDelta = 0.001;
    private final static double kDt = 0.02; // s
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());
    private final static SwerveKinodynamics kKinematicLimits = SwerveKinodynamicsFactory.limiting();

    private final static double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
    private final static double kMaxAccelerationError = 0.1; // m/s^2

    private void SatisfiesConstraints(int iteration, SwerveSetpoint prev, SwerveSetpoint next) {
        final SwerveModuleState100[] prevStates = prev.states().all();
        final SwerveModuleState100[] nextStates = next.states().all();
        for (int i = 0; i < prevStates.length; ++i) {
            final SwerveModuleState100 prevModule = prevStates[i];
            final SwerveModuleState100 nextModule = nextStates[i];
            Optional<Rotation2d> prevAngleOpt = prevModule.angle();
            Optional<Rotation2d> nextAngleOpt = nextModule.angle();
            Rotation2d diffRotation = GeometryUtil.kRotationZero;
            if (prevAngleOpt.isPresent() && nextAngleOpt.isPresent()) {
                diffRotation = prevAngleOpt.get().unaryMinus().rotateBy(nextAngleOpt.get());
            }
            assertTrue(
                    Math.abs(diffRotation.getRadians()) < kKinematicLimits.getMaxSteeringVelocityRad_S()
                            + kMaxSteeringVelocityError,
                    String.format("%d %d %f %f %f", iteration, i,
                            diffRotation.getRadians(),
                            kKinematicLimits.getMaxSteeringVelocityRad_S(),
                            kMaxSteeringVelocityError));
            assertTrue(Math.abs(nextModule.speedMetersPerSecond()) <= kKinematicLimits.getMaxDriveVelocityM_S(),
                    String.format("%d %d %f %f", iteration, i,
                            nextModule.speedMetersPerSecond(),
                            kKinematicLimits.getMaxDriveVelocityM_S()));
            double actual = Math.abs(
                    nextModule.speedMetersPerSecond() - prevModule.speedMetersPerSecond())
                    / kDt;
            double limit = kKinematicLimits.getMaxDriveAccelerationM_S2() + kMaxAccelerationError;
            assertTrue(actual <= limit,
                    String.format("%d %d %f %f %f %f %f %f", iteration, i, actual, limit,
                            nextModule.speedMetersPerSecond(),
                            prevModule.speedMetersPerSecond(),
                            kKinematicLimits.getMaxDriveAccelerationM_S2(),
                            kMaxAccelerationError));
        }
    }

    private SwerveSetpoint driveToGoal(
            SwerveSetpoint prevSetpoint,
            ChassisSpeeds goal,
            AsymSwerveSetpointGenerator generator) {
        int iteration = 0;
        while (GeometryUtil.norm(goal.minus(prevSetpoint.speeds())) > 1e-6) {
            SwerveSetpoint newsetpoint = generator.generateSetpoint(prevSetpoint, goal);
            SatisfiesConstraints(iteration, prevSetpoint, newsetpoint);
            prevSetpoint = newsetpoint;
            ++iteration;
        }
        return prevSetpoint;
    }

    /** The setpoint generator never changes the field-relative course. */
    @Test
    void courseInvariant() {
        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                kKinematicLimits,
                () -> 12);
        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.generateSetpoint(
                prevSetpoint,
                target);

    }

    @Test
    void motionlessNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                unlimited,
                () -> 12);

        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 0);
        Rotation2d theta = new Rotation2d();
        // this is the instantaneous speed
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);

        SwerveModuleStates targetStates = unlimited.toSwerveModuleStates(target);
        assertEquals(0, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(0, target.omegaRadiansPerSecond, kDelta);
        assertEquals(0, targetStates.frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, targetStates.frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(0, targetStates.rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, targetStates.rearRight().speedMetersPerSecond(), kDelta);
        assertTrue(targetStates.frontLeft().angle().isEmpty());
        assertTrue(targetStates.frontRight().angle().isEmpty());
        assertTrue(targetStates.rearLeft().angle().isEmpty());
        assertTrue(targetStates.rearRight().angle().isEmpty());

        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.generateSetpoint(prevSetpoint, target);
        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
        assertEquals(0, setpoint.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, setpoint.states().frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(0, setpoint.states().rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, setpoint.states().rearRight().speedMetersPerSecond(), kDelta);
        assertTrue(setpoint.states().frontLeft().angle().isEmpty());
        assertTrue(setpoint.states().frontRight().angle().isEmpty());
        assertTrue(setpoint.states().rearLeft().angle().isEmpty());
        assertTrue(setpoint.states().rearRight().angle().isEmpty());
    }

    @Test
    void driveNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                unlimited,
                () -> 12);

        FieldRelativeVelocity v = new FieldRelativeVelocity(1, 0, 0);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);

        SwerveModuleStates targetStates = unlimited.toSwerveModuleStates(target);
        assertEquals(1, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(0, target.omegaRadiansPerSecond, kDelta);
        assertEquals(1, targetStates.frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(1, targetStates.frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(1, targetStates.rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(1, targetStates.rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(0, targetStates.frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0, targetStates.frontRight().angle().get().getRadians(), kDelta);
        assertEquals(0, targetStates.rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(0, targetStates.rearRight().angle().get().getRadians(), kDelta);

        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.generateSetpoint(prevSetpoint, target);
        assertEquals(1, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
        assertEquals(1, setpoint.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(1, setpoint.states().frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(1, setpoint.states().rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(1, setpoint.states().rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(0, setpoint.states().frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0, setpoint.states().frontRight().angle().get().getRadians(), kDelta);
        assertEquals(0, setpoint.states().rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(0, setpoint.states().rearRight().angle().get().getRadians(), kDelta);
    }

    @Test
    void spinNoOp() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                unlimited,
                () -> 12);

        FieldRelativeVelocity v = new FieldRelativeVelocity(0, 0, 1);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);

        SwerveModuleStates targetStates = unlimited.toSwerveModuleStates(target);
        assertEquals(0, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(1, target.omegaRadiansPerSecond, kDelta);
        assertEquals(0.353, targetStates.frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, targetStates.frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, targetStates.rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, targetStates.rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(2.356, targetStates.frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0.785, targetStates.frontRight().angle().get().getRadians(), kDelta);
        assertEquals(-2.356, targetStates.rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(-0.785, targetStates.rearRight().angle().get().getRadians(), kDelta);

        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.generateSetpoint(prevSetpoint, target);
        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(1, setpoint.speeds().omegaRadiansPerSecond, kDelta);
        assertEquals(0.353, setpoint.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, setpoint.states().frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, setpoint.states().rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0.353, setpoint.states().rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(2.356, setpoint.states().frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0.785, setpoint.states().frontRight().angle().get().getRadians(), kDelta);
        assertEquals(-2.356, setpoint.states().rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(-0.785, setpoint.states().rearRight().angle().get().getRadians(), kDelta);
    }

    @Test
    void driveAndSpin() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.unlimited();
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                unlimited,
                () -> 12);

        // spin fast to make the discretization effect larger
        FieldRelativeVelocity v = new FieldRelativeVelocity(5, 0, 25);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);

        SwerveModuleStates targetStates = unlimited.toSwerveModuleStates(target);
        assertEquals(5, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(25, target.omegaRadiansPerSecond, kDelta);
        assertEquals(3.833, targetStates.frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(9.780, targetStates.frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(10.566, targetStates.rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(13.878, targetStates.rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(2.474, targetStates.frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0.245, targetStates.frontRight().angle().get().getRadians(), kDelta);
        assertEquals(-1.859, targetStates.rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(-0.818, targetStates.rearRight().angle().get().getRadians(), kDelta);

        // this should do nothing since the limits are so high
        SwerveSetpoint prevSetpoint = new SwerveSetpoint();
        SwerveSetpoint setpoint = generator.generateSetpoint(prevSetpoint, target);
        assertEquals(5, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(25, setpoint.speeds().omegaRadiansPerSecond, kDelta);
        ChassisSpeeds implied = unlimited.toChassisSpeedsWithDiscretization(setpoint.states(), 0.02);
        assertEquals(5, implied.vxMetersPerSecond, kDelta);
        assertEquals(0, implied.vyMetersPerSecond, kDelta);
        assertEquals(25, implied.omegaRadiansPerSecond, kDelta);
        assertEquals(3.833, setpoint.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(9.780, setpoint.states().frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(10.566, setpoint.states().rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(13.878, setpoint.states().rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(2.474, setpoint.states().frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0.245, setpoint.states().frontRight().angle().get().getRadians(), kDelta);
        assertEquals(-1.859, setpoint.states().rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(-0.818, setpoint.states().rearRight().angle().get().getRadians(), kDelta);
    }

    @Test
    void driveAndSpinLimited() {
        SwerveKinodynamics unlimited = SwerveKinodynamicsFactory.limiting();
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                unlimited,
                () -> 12);

        // spin fast to make the discretization effect larger
        FieldRelativeVelocity v = new FieldRelativeVelocity(5, 0, 25);
        Rotation2d theta = new Rotation2d();
        ChassisSpeeds target = SwerveKinodynamics.toInstantaneousChassisSpeeds(v, theta);

        SwerveModuleStates targetStates = unlimited.toSwerveModuleStates(target);
        assertEquals(5, target.vxMetersPerSecond, kDelta);
        assertEquals(0, target.vyMetersPerSecond, kDelta);
        assertEquals(25, target.omegaRadiansPerSecond, kDelta);
        assertEquals(3.833, targetStates.frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(9.780, targetStates.frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(10.566, targetStates.rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(13.878, targetStates.rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(2.474, targetStates.frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0.245, targetStates.frontRight().angle().get().getRadians(), kDelta);
        assertEquals(-1.859, targetStates.rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(-0.818, targetStates.rearRight().angle().get().getRadians(), kDelta);

        // the spin is 5x the drive, as requested.
        // use the target as the previous setpoint
        SwerveSetpoint prevSetpoint = new SwerveSetpoint(target, targetStates);
        SwerveSetpoint setpoint = generator.generateSetpoint(prevSetpoint, target);
        assertEquals(4.836, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(24.183, setpoint.speeds().omegaRadiansPerSecond, kDelta);
        ChassisSpeeds implied = unlimited.toChassisSpeedsWithDiscretization(setpoint.states(), 0.02);
        assertEquals(4.836, implied.vxMetersPerSecond, kDelta);
        assertEquals(0, implied.vyMetersPerSecond, kDelta);
        assertEquals(24.183, implied.omegaRadiansPerSecond, kDelta);
        // nowhere near max speed
        assertEquals(3.687, setpoint.states().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(9.584, setpoint.states().frontRight().speedMetersPerSecond(), kDelta);
        assertEquals(10.102, setpoint.states().rearLeft().speedMetersPerSecond(), kDelta);
        assertEquals(13.428, setpoint.states().rearRight().speedMetersPerSecond(), kDelta);
        assertEquals(2.437, setpoint.states().frontLeft().angle().get().getRadians(), kDelta);
        assertEquals(0.252, setpoint.states().frontRight().angle().get().getRadians(), kDelta);
        assertEquals(-1.852, setpoint.states().rearLeft().angle().get().getRadians(), kDelta);
        assertEquals(-0.807, setpoint.states().rearRight().angle().get().getRadians(), kDelta);
    }

    @Test
    void testGenerateSetpoint() {
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())));
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);
        AsymSwerveSetpointGenerator generator = new AsymSwerveSetpointGenerator(
                logger,
                kKinematicLimits,
                () -> 12);

        ChassisSpeeds goalSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 0.0, -1.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.1, -1.0, 0.0);
        // because this is a u-turn, first the robot stops, and then it proceeds in the
        // new direction; there is an empty angle at the stopping point.
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, -0.5, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, 0.4, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);
    }

    @Test
    void testSpeedLimit() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.limiting();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger,
                limits,
                () -> 12);

        // just one too-fast module
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(10, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // here the setpoint is at angle zero

        // desired speed is even faster
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(10.02, 0, 0);

        // since the desired speed is too fast, we don't do anything.
        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.163, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testLimiting() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.limiting();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger,
                limits,
                () -> 12);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // here the setpoint is at angle zero

        // desired speed is very fast
        // omega is zero since it's hard to deal with in a test.
        FieldRelativeVelocity desiredV = new FieldRelativeVelocity(10, 10, 0);
        ChassisSpeeds desiredSpeeds = SwerveKinodynamics.toInstantaneousChassisSpeeds(desiredV, new Rotation2d());

        // initially it's not moving fast at all
        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        // note this says the angles are all empty which is wrong, they should be the
        // previous values.

        // steer in place for a short time.
        for (int i = 0; i < 6; ++i) {
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
            assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
            assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        }

        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        // after 1 second, it's going faster.
        for (int i = 0; i < 50; ++i) {
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        }
        assertEquals(3.508, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(3.508, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    // simple accel case: are we limiting the right amount?
    @Test
    void testAccel() {
        // limit accel is 10 m/s^2
        // capsize limit is 24.5 m/s^2
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        assertEquals(24.5, limits.getMaxCapsizeAccelM_S2(), kDelta);
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger,
                limits,
                () -> 12);

        // initially at rest, wheels facing forward.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // initial setpoint steering is at angle zero

        // desired speed +x
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(10, 0, 0);

        // the first setpoint should be accel limited: 10 m/s^2, 0.02 sec,
        // so v = 0.2 m/s
        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        // note this says the angles are all empty which is wrong, they should be the
        // previous values.

        // after 1 second, it's going faster.
        for (int i = 0; i < 50; ++i) {
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        }
        assertEquals(4.9, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testNotLimiting() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger,
                limits,
                () -> 12);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is feasible, max accel = 10 * dt = 0.02 => v = 0.2
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.2, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testLimitingALittle() {
        // high centripetal limit to stay out of the way
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(logger, limits,
                () -> 12);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is double the feasible accel so we should reach it in two
        // iterations.
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.4, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.2, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.4, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testLowCentripetal() {
        // very low centripetal limit so we can see it
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.lowCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger, limits, () -> 12);

        // initially at rest.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is double the feasible accel so we should reach it in two
        // iterations.
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0.4, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.024, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0.049, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    /**
     * This starts full speed +x, and wants full speed +y.
     * 
     * The main purpose of this test is to print the path.
     */
    @Test
    void testCentripetal() {
        boolean dump = false;
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.limiting();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(logger, limits,
                () -> 12);

        // initially moving full speed +x
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(4, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(4, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        assertEquals(4, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);

        // desired state is full speed +y
        final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 4, 0);

        SwerveSetpoint prev = setpoint;
        Pose2d currentPose = GeometryUtil.kPoseZero;
        if (dump)
            Util.printf("i     x     y    vx    vy drive steer     ax    ay      a\n");

        // first slow from 4 m/s to 0 m/s stop at 10 m/s^2, so 0.4s
        for (int i = 0; i < 50; ++i) {
            Twist2d discrete = GeometryUtil.discretize(setpoint.speeds(), kDt);
            currentPose = currentPose.exp(discrete);
            setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);

            double ax = (setpoint.speeds().vxMetersPerSecond - prev.speeds().vxMetersPerSecond)
                    / kDt;
            double ay = (setpoint.speeds().vyMetersPerSecond - prev.speeds().vyMetersPerSecond)
                    / kDt;
            double a = Math.hypot(ax, ay);

            if (dump)
                Util.printf("%d %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n",
                        i, currentPose.getX(), currentPose.getY(),
                        setpoint.speeds().vxMetersPerSecond,
                        setpoint.speeds().vyMetersPerSecond,
                        setpoint.states().frontLeft().speedMetersPerSecond(),
                        setpoint.states().frontLeft().angle().get().getRadians(),
                        ax, ay, a);
            prev = setpoint;
        }

        // we end up going the right way
        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(4, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testCase4() {
        // this corresponds to the "4" cases in SwerveUtilTest.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.decelCase();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(logger, limits,
                () -> 12);

        // initially moving 0.5 +y
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0.5, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)),
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)),
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)),
                new SwerveModuleState100(0.5, Optional.of(GeometryUtil.kRotation90)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired state is 1 +x
        final ChassisSpeeds desiredSpeeds = new ChassisSpeeds(1, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);

        // so one iteration should yield the same values as in SwerveUtilTest,
        // where the governing constraint was the steering one, s = 0.048.
        assertEquals(0.048, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0.476, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    /**
     * What happens when the setpoint is too fast, the setpoint generator tries to
     * slow down without violating the decel and centripetal constraints.
     */
    @Test
    void testOverspeed() {
        // very high decel and centripetal limit allows immediate reduction to max
        // allowed speed.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.highDecelAndCapsize();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger, limits, () -> 12);

        // initial speed is faster than possible.
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(10, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(10, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(10, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(10, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(10, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is faster than possible.
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(10, 0, 0);

        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        // desaturation limits the speed to max V
        assertEquals(5, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testOverspeedCentripetal() {
        // very high decel and centripetal limit allows immediate reduction to max
        // allowed speed.
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.get();
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger, limits, () -> 12);

        // initial speed is at the limit +x
        ChassisSpeeds initialSpeeds = new ChassisSpeeds(5, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(5, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(5, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(5, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(5, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        // desired speed is at the limit +y
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 5, 0);

        // the turn is pretty slow
        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(4.653, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0.346, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }

    @Test
    void testBrownout() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.get();
        // shouldn't allow any movement at 6v.
        AsymSwerveSetpointGenerator swerveSetpointGenerator = new AsymSwerveSetpointGenerator(
                logger, limits, () -> 6);

        ChassisSpeeds initialSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleStates initialStates = new SwerveModuleStates(
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)),
                new SwerveModuleState100(0, Optional.of(GeometryUtil.kRotationZero)));
        SwerveSetpoint setpoint = new SwerveSetpoint(initialSpeeds, initialStates);

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(1, 0, 0);

        // no output allowed
        setpoint = swerveSetpointGenerator.generateSetpoint(setpoint, desiredSpeeds);
        assertEquals(0, setpoint.speeds().vxMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().vyMetersPerSecond, kDelta);
        assertEquals(0, setpoint.speeds().omegaRadiansPerSecond, kDelta);
    }
}