package org.team100.lib.commands.drivetrain.for_testing;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.commands.drivetrain.for_testing.DriveInALittleSquare.DriveState;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveModuleState100;
import org.team100.lib.state.Control100;
import org.team100.lib.testing.Timeless;
import org.team100.lib.util.Util;

class DriveInALittleSquareTest extends Fixtured implements Timeless {
    boolean dump = false;
    private static final double kDelta = 0.001;

    /** Confirm that the steering commands are simple steps. */
    @Test
    void testSteering() {
        SwerveDriveSubsystem swerve = fixture.drive;
        DriveInALittleSquare command = new DriveInALittleSquare(swerve);
        command.initialize();
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        // we're already pointing the right way
        stepTime(0.02);
        fixture.drive.periodic();
        command.execute();
        // execute switches to driving
        assertEquals(DriveInALittleSquare.DriveState.DRIVING, command.m_state);
        // and changes the setpoint
        assertEquals(0.02, command.m_setpoint.v(), kDelta);
        assertEquals(0, command.m_goal.getRadians(), kDelta);
        // and sets the desired state (note this is not reflected in the measurement)
        assertEquals(0.02, swerve.getSwerveLocal().getDesiredStates().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(0, swerve.getSwerveLocal().getDesiredStates().frontLeft().angle().get().getRadians(), kDelta);

        // step through the driving phase
        for (int i = 0; i < 98; ++i) {
            stepTime(0.02);
            fixture.drive.periodic();
            command.execute();
            Util.printf("************* %d STATE %s *************\n", i, command.m_state);
            Util.printf("%f\n",swerve.getSwerveLocal().getDesiredStates().frontLeft().speedMetersPerSecond());
        }
        // now we should be steering again
        stepTime(0.02);
        fixture.drive.periodic();
        command.execute();
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        assertEquals(0, command.m_setpoint.v(), kDelta);
        assertEquals(Math.PI / 2, command.m_goal.getRadians(), kDelta);
        assertEquals(0, swerve.getSwerveLocal().getDesiredStates().frontLeft().speedMetersPerSecond(), kDelta);
        assertEquals(Math.PI / 2, swerve.getSwerveLocal().getDesiredStates().frontLeft().angle().get().getRadians(),
                kDelta);
    }

    @Test
    void testLowLevel() {
        // there's some weird thing with getFPGATimestamp in tests mode
        // that messes up how the simulated encoder measures position,
        // and somehow magically the line below fixes it.
        // assertEquals(0.0, fixture.drive.getSwerveLocal().positions().frontLeft().distanceMeters, 0.005);

        DriveInALittleSquare command = new DriveInALittleSquare(fixture.drive);
        command.initialize();

        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);

        stepTime(0.02);
        fixture.drive.periodic();
        command.execute();
        assertEquals(DriveInALittleSquare.DriveState.DRIVING, command.m_state);
        assertEquals(0, command.m_goal.getRadians(), kDelta);

        // a little while later we should be driving
        // at this point the speed is still zero
        assertEquals(0.0, fixture.drive.getSwerveLocal().states().frontLeft().speedMetersPerSecond(), 0.005);
        stepTime(0.1);
        for (int i = 0; i < 5; ++i) {
            stepTime(0.02);
            fixture.drive.periodic();
            // this changes the speed
            command.execute();
        }
        // big jump in speed here since dt is so big
        // max accel is 1 so waiting 0.1s means 0.1m/s
        assertEquals(DriveInALittleSquare.DriveState.DRIVING, command.m_state);
        assertEquals(0.1, command.m_setpoint.v(), 0.05);
        assertEquals(0, command.m_goal.getRadians(), kDelta);
        assertEquals(0.1, fixture.drive.getSwerveLocal().states().frontLeft().speedMetersPerSecond(), 0.005);
        // position isn't updated until the next time-step.
        // assertEquals(0.0,
        // fixture.drive.getSwerveLocal().positions().frontLeft().distanceMeters,
        // 0.005);

        // drive to the next corner. at 1 m/s/s this should be a triangular
        // profile that takes exactly 2 sec total but we started at 0.1 so 1.9
        for (double t = 0; t < 1.9; t += 0.02) {
            stepTime(0.02);
            fixture.drive.periodic();
            command.execute();
            double speed = fixture.drive.getSwerveLocal().states().frontLeft().speedMetersPerSecond();
            double distance = fixture.drive.getSwerveLocal().positions().frontLeft().distanceMeters;
            DriveState state = command.m_state;
            if (dump)
                Util.printf("t %5.3f state %s speed %5.3f distance %5.3f\n", t, state.toString(), speed, distance);
        }

        // steer
        stepTime(0.02);
        fixture.drive.periodic();
        command.execute();
        assertEquals(DriveInALittleSquare.DriveState.STEERING, command.m_state);
        assertEquals(0, command.m_setpoint.v(), kDelta);
        assertEquals(Math.PI / 2, command.m_goal.getRadians(), kDelta);
        assertEquals(0, fixture.drive.getSwerveLocal().states().frontLeft().speedMetersPerSecond(), kDelta);
        assertFalse(fixture.drive.getSwerveLocal().atGoal()[0]);

        // wait a half second.
        for (double t = 0; t < 0.5; t += 0.02) {
            stepTime(0.02);
            fixture.drive.periodic();
            command.execute();
            double measurement = fixture.drive.getSwerveLocal().states().frontLeft().angle().get().getRadians();
            SwerveModuleState100 goal = fixture.swerveLocal.getDesiredStates().frontLeft();
            Control100 setpoint = fixture.swerveLocal.getSetpoints()[0];
            // this output is useful to see what's happening.
            if (dump)
                Util.printf("t %5.3f goal %5.3f setpoint x %5.3f setpoint v %5.3f measurement %5.3f\n",
                        t,
                        goal.angle().get().getRadians(),
                        setpoint.x(),
                        setpoint.v(),
                        measurement);
        }
        // after that time, the wheels have rotated.
        // note the controller tolerance is
        assertEquals(Math.PI / 2, fixture.drive.getSwerveLocal().states().frontLeft().angle().get().getRadians(), 0.01);
        assertTrue(fixture.drive.getSwerveLocal().atGoal()[0]);
        // and we're driving again
        assertEquals(DriveInALittleSquare.DriveState.DRIVING, command.m_state);
        // we're not quite motionless, we're already going a little.
        // there's no specific test here because the velocity seems to depend
        // on the timing in the simulation
        assertTrue(command.m_setpoint.v() > 0);
    }

}
