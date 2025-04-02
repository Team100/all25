package org.team100.lib.commands.drivetrain.for_testing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.testing.Timeless;

class DriveRotationTest extends Fixtured implements Timeless {
    private static final double kDelta = 0.001;
    private final double desiredRotation = 1;

    @Test
    void testSimple() {
        Supplier<Double> rot = () -> desiredRotation;
        DriveRotation command = new DriveRotation(fixture.drive, rot);
        command.initialize();
        assertEquals(0, fixture.drive.getPose().getX(), kDelta);
        command.execute();
        command.end(false);
    }
}
