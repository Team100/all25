package org.team100.lib.motion.drivetrain;

import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.localization.SwerveDrivePoseEstimator100;
import org.team100.lib.localization.VisionData;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.sensors.SimulatedGyro;
import org.team100.lib.testing.Timeless;

/**
 * A real swerve subsystem populated with simulated motors and encoders,
 * for testing.
 * 
 * Uses simulated position sensors, must be used with clock control (e.g.
 * {@link Timeless}).
 */
public class RealisticFixture {
    public SwerveModuleCollection collection;
    public Gyro gyro;
    public SwerveDrivePoseEstimator100 poseEstimator;
    public SwerveKinodynamics swerveKinodynamics;
    public SwerveLocal swerveLocal;
    public SwerveDriveSubsystem drive;
    public SwerveController controller;
    public LoggerFactory logger;
    public LoggerFactory fieldLogger;

    public RealisticFixture() {
        logger = new TestLoggerFactory(new TestPrimitiveLogger());
        fieldLogger = new TestLoggerFactory(new TestPrimitiveLogger());
        swerveKinodynamics = SwerveKinodynamicsFactory.forRealisticTest();
        collection = SwerveModuleCollection.get(logger, 10, 20, swerveKinodynamics);
        gyro = new SimulatedGyro(swerveKinodynamics, collection);
        swerveLocal = new SwerveLocal(logger, swerveKinodynamics, collection);
        poseEstimator = swerveKinodynamics.newPoseEstimator(
                logger,
                gyro,
                collection.positions(),
                GeometryUtil.kPoseZero,
                0); // initial time is zero here for testing
        VisionData v = new VisionData() {
            @Override
            public void update() {
            }
        };
        SwerveLimiter limiter = new SwerveLimiter(swerveKinodynamics, () -> 12);

        drive = new SwerveDriveSubsystem(
                fieldLogger,
                logger,
                gyro,
                poseEstimator,
                swerveLocal,
                v,
                limiter);

        controller = SwerveControllerFactory.test(logger);
    }

    public void close() {
        // close the DIO inside the turning encoder
        collection.close();
    }

}
