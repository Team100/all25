package org.team100.frc2025.CalgamesArm;

import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.CombinedRotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.GearedRotaryPositionSensor;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.geometry.GlobalAccelerationR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.ConfigLogger;
import org.team100.lib.logging.LoggerFactory.GlobalAccelerationR3Logger;
import org.team100.lib.logging.LoggerFactory.GlobalVelocityR3Logger;
import org.team100.lib.logging.LoggerFactory.JointAccelerationsLogger;
import org.team100.lib.logging.LoggerFactory.JointForceLogger;
import org.team100.lib.logging.LoggerFactory.JointVelocitiesLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.Config;
import org.team100.lib.motion.kinematics.AnalyticalJacobian;
import org.team100.lib.motion.kinematics.ElevatorArmWristKinematics;
import org.team100.lib.motion.kinematics.JointAccelerations;
import org.team100.lib.motion.kinematics.JointForce;
import org.team100.lib.motion.kinematics.JointVelocities;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.motor.Talon6Motor;
import org.team100.lib.music.Music;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.team100.lib.util.StrUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CalgamesMech extends SubsystemBase implements Music {
    private static final boolean DEBUG = false;
    private boolean DISABLED = false;
    ////////////////////////////////////////////////////////
    ///
    /// CANONICAL CONFIGS
    /// These are used with profiles.
    ///
    private static final Config HOME = new Config(0, 0, 0);
    private static final Config CORAL_GROUND_PICK = new Config(0, -1.83, -0.12);
    private static final Config CLIMB = new Config(0, -1.83, 2);
    private static final Config STATION = new Config(0, -1, 0);
    private static final Config PROCESSOR = new Config(0, 1.2, 0);
    private static final Config ALGAE_GROUND = new Config(0, 1.43, 0);
    private static final Config L1 = new Config(0, -.95, -.5);

    ////////////////////////////////////////////////////////
    ///
    /// CANONICAL POSES
    /// These are used with trajectories.
    ///
    private static final Pose2d L2 = new Pose2d(0.56, 0.54, rad(2.0));
    private static final Pose2d L3 = new Pose2d(0.94, 0.56, rad(1.7));
    private static final Pose2d L4 = new Pose2d(1.57, 0.54, rad(2.0));
    private static final Pose2d ALGAE_L2 = new Pose2d(0.85, 0.7, rad(1.5));
    private static final Pose2d ALGAE_L3 = new Pose2d(1.15, 0.7, rad(1.5));
    private static final Pose2d BARGE = new Pose2d(2.3, -0.5, rad(-1.5));

    private final double m_armLengthM;
    private final double m_wristLengthM;
    private final MechTrajectories m_transit;

    private final ElevatorArmWristKinematics m_kinematics;
    private final AnalyticalJacobian m_jacobian;

    private final Dynamics m_dynamics;

    private final ConfigLogger m_log_config;
    private final JointVelocitiesLogger m_log_jointV;
    private final JointAccelerationsLogger m_log_jointA;
    private final JointForceLogger m_log_jointF;

    private final Pose2dLogger m_log_pose;
    private final GlobalVelocityR3Logger m_log_cartesianV;
    private final GlobalAccelerationR3Logger m_log_cartesianA;

    private final LinearMechanism m_elevatorFront;
    private final LinearMechanism m_elevatorBack;

    private final RotaryMechanism m_shoulder;
    private final RotaryMechanism m_wrist;

    /** Home pose is Config(0,0,0), from forward kinematics. */
    private final Pose2d m_home;

    private final List<Talon6Motor> m_players;

    public CalgamesMech(
            LoggerFactory log,
            double armLength,
            double wristLength) {
        LoggerFactory parent = log.type(this);
        m_armLengthM = armLength;
        m_wristLengthM = wristLength;

        m_kinematics = new ElevatorArmWristKinematics(armLength, wristLength);
        m_jacobian = new AnalyticalJacobian(m_kinematics);
        m_dynamics = new Dynamics();

        m_home = m_kinematics.forward(HOME);

        m_transit = new MechTrajectories(this, m_kinematics, m_jacobian);

        LoggerFactory jointLog = parent.name("joints");
        m_log_config = jointLog.logConfig(Level.DEBUG, "config");
        m_log_jointV = jointLog.logJointVelocities(Level.DEBUG, "velocity");
        m_log_jointA = jointLog.logJointAccelerations(Level.DEBUG, "accel");
        m_log_jointF = jointLog.logJointForce(Level.DEBUG, "force");

        LoggerFactory cartesianLog = parent.name("cartesian");
        m_log_pose = cartesianLog.pose2dLogger(Level.DEBUG, "pose");
        m_log_cartesianV = cartesianLog.globalVelocityR3Logger(Level.DEBUG, "velocity");
        m_log_cartesianA = cartesianLog.globalAccelerationR3Logger(Level.DEBUG, "accel");

        LoggerFactory elevatorbackLog = parent.name("elevatorBack");
        LoggerFactory elevatorfrontLog = parent.name("elevatorFront");
        LoggerFactory shoulderLog = parent.name("shoulder");
        LoggerFactory wristLog = parent.name("wrist");

        m_players = new ArrayList<>();

        switch (Identity.instance) {
            case COMP_BOT -> {

                final double elevatorGearRatio = 2.182;
                final double elevatorDrivePulleyDiameterM = 0.03844;
                final double elevatorLowerLimit = 0;
                final double elevatorUpperLimit = 2.1;

                Kraken6Motor elevatorFrontMotor = new Kraken6Motor(
                        elevatorfrontLog,
                        new CanId(11),
                        NeutralMode.BRAKE, MotorPhase.REVERSE,
                        100,
                        100,
                        PIDConstants.makePositionPID(5),
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                m_players.add(elevatorFrontMotor);
                Talon6Encoder elevatorFrontEncoder = new Talon6Encoder(
                        elevatorfrontLog, elevatorFrontMotor);

                m_elevatorFront = new LinearMechanism(
                        elevatorfrontLog, elevatorFrontMotor, elevatorFrontEncoder,
                        elevatorGearRatio, elevatorDrivePulleyDiameterM,
                        elevatorLowerLimit, elevatorUpperLimit);

                Kraken6Motor elevatorBackMotor = new Kraken6Motor(
                        elevatorbackLog,
                        new CanId(12),
                        NeutralMode.BRAKE, MotorPhase.FORWARD,
                        100, // orginally 60
                        100, // originally 90
                        PIDConstants.makePositionPID(5),
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                m_players.add(elevatorBackMotor);
                Talon6Encoder elevatorBackEncoder = new Talon6Encoder(
                        elevatorbackLog, elevatorBackMotor);
                m_elevatorBack = new LinearMechanism(
                        elevatorbackLog, elevatorBackMotor, elevatorBackEncoder,
                        elevatorGearRatio, elevatorDrivePulleyDiameterM,
                        elevatorLowerLimit, elevatorUpperLimit);

                Kraken6Motor shoulderMotor = new Kraken6Motor(
                        shoulderLog,
                        new CanId(24),
                        NeutralMode.BRAKE,
                        MotorPhase.REVERSE,
                        100, // og 60
                        100, // og 90
                        PIDConstants.makePositionPID(5),
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                m_players.add(shoulderMotor);
                Talon6Encoder shoulderEncoder = new Talon6Encoder(
                        shoulderLog, shoulderMotor);
                // The shoulder has a 5048 on the intermediate shaft
                AS5048RotaryPositionSensor shoulderSensor = new AS5048RotaryPositionSensor(
                        shoulderLog,
                        new RoboRioChannel(4),
                        0.684, // <<< This is the input offset (in TURNS) to adjust when zeroing
                        EncoderDrive.INVERSE);
                GearedRotaryPositionSensor gearedSensor = new GearedRotaryPositionSensor(
                        shoulderSensor,
                        8);

                ProxyRotaryPositionSensor shoulderProxySensor = new ProxyRotaryPositionSensor(
                        shoulderEncoder,
                        78);
                CombinedRotaryPositionSensor shoulderCombined = new CombinedRotaryPositionSensor(
                        shoulderLog, gearedSensor, shoulderProxySensor);
                m_shoulder = new RotaryMechanism(
                        shoulderLog, shoulderMotor, shoulderCombined,
                        78,
                        -2,
                        2);

                Kraken6Motor wristMotor = new Kraken6Motor(
                        wristLog,
                        new CanId(22),
                        NeutralMode.BRAKE, MotorPhase.FORWARD,
                        40, // og 60
                        60, // og 90
                        PIDConstants.makePositionPID(8), // og 10
                        Feedforward100.makeWCPSwerveTurningFalcon6());
                m_players.add(wristMotor);
                // the wrist has no angle sensor, so it needs to start in the "zero" position.
                Talon6Encoder wristEncoder = new Talon6Encoder(
                        wristLog, wristMotor);
                final double wristGearRatio = 55.710;
                ProxyRotaryPositionSensor wristProxySensor = new ProxyRotaryPositionSensor(
                        wristEncoder,
                        wristGearRatio);
                double wristEncoderOffset = 2.06818; // 2+0.06818
                wristProxySensor.setEncoderPosition(wristEncoderOffset);
                m_wrist = new RotaryMechanism(
                        wristLog, wristMotor, wristProxySensor, wristGearRatio,
                        -1.5, // min
                        2.1); // max
            }
            default -> {
                SimulatedBareMotor elevatorMotorFront = new SimulatedBareMotor(
                        elevatorfrontLog, 600);
                SimulatedBareEncoder elevatorEncoderFront = new SimulatedBareEncoder(
                        elevatorfrontLog,
                        elevatorMotorFront);
                m_elevatorFront = new LinearMechanism(
                        elevatorfrontLog, elevatorMotorFront, elevatorEncoderFront,
                        2, 0.05, 0, 2.2);

                SimulatedBareMotor elevatorMotorBack = new SimulatedBareMotor(
                        elevatorbackLog, 600);
                SimulatedBareEncoder elevatorEncoderBack = new SimulatedBareEncoder(
                        elevatorbackLog, elevatorMotorBack);
                m_elevatorBack = new LinearMechanism(
                        elevatorbackLog, elevatorMotorBack, elevatorEncoderBack,
                        2, 0.05, 0, 2.2);

                SimulatedBareMotor shoulderMotor = new SimulatedBareMotor(
                        shoulderLog, 600);
                SimulatedBareEncoder shoulderEncoder = new SimulatedBareEncoder(
                        shoulderLog, shoulderMotor);
                RotaryPositionSensor shoulderSensor = new SimulatedRotaryPositionSensor(
                        shoulderLog, shoulderEncoder, 100);
                m_shoulder = new RotaryMechanism(
                        shoulderLog, shoulderMotor, shoulderSensor, 100, -3, 3);

                SimulatedBareMotor wristMotor = new SimulatedBareMotor(
                        wristLog, 600);
                SimulatedBareEncoder wristEncoder = new SimulatedBareEncoder(
                        wristLog, wristMotor);
                RotaryPositionSensor wristSensor = new SimulatedRotaryPositionSensor(
                        wristLog, wristEncoder, 58);
                m_wrist = new RotaryMechanism(
                        wristLog, wristMotor, wristSensor, 58, -3, 3);
            }
        }
    }

    @Override
    public Command play(double freq) {
        return run(() -> {
            for (Talon6Motor m : m_players) {
                m.play(freq);
            }
        });
    }

    public double getArmLength() {
        return m_armLengthM;
    }

    public double getHandLength() {
        return m_wristLengthM;
    }

    public Config getConfig() {
        return new Config(
                m_elevatorBack.getPositionM(),
                m_shoulder.getWrappedPositionRad(),
                m_wrist.getWrappedPositionRad());
    }

    public JointVelocities getJointVelocity() {
        return new JointVelocities(
                m_elevatorBack.getVelocityM_S(),
                m_shoulder.getVelocityRad_S(),
                m_wrist.getVelocityRad_S());
    }

    public ModelR3 getState() {
        Config c = getConfig();
        JointVelocities jv = getJointVelocity();
        Pose2d p = m_kinematics.forward(c);
        GlobalVelocityR3 v = m_jacobian.forward(c, jv);
        return new ModelR3(p, v);
    }

    /** There are no profiles here, so this control needs to be feasible. */
    public void set(ControlR3 control) {
        Pose2d pose = control.pose();
        Config config = m_kinematics.inverse(pose);
        if (DEBUG) {
            System.out.printf("pose %s config %s\n", StrUtil.pose2Str(pose), config);
        }
        if (config.isNaN()) {
            if (DEBUG)
                System.out.println("skipping invalid config");
            stop();
            return;
        }
        JointVelocities jv = m_jacobian.inverse(control.model());
        JointAccelerations ja = m_jacobian.inverseA(control);
        set(config, jv, ja);
    }

    public void set(Config c, JointVelocities jv, JointAccelerations ja) {
        JointForce jf = m_dynamics.forward(c, jv, ja);
        set(c, jv, ja, jf);
    }

    /** This is not "hold position" this is "torque off". */
    public void stop() {
        m_elevatorFront.stop();
        m_elevatorBack.stop();
        m_shoulder.stop();
        m_wrist.stop();
    }

    /////////////////////////////////////////////////////////
    ///
    /// PROFILE COMMANDS
    ///

    /**
     * Use a profile to move from the current position and velocity to the "home"
     * position (origin) at rest, and end when done.
     */
    public Command profileHomeTerminal() {
        FollowJointProfiles f = FollowJointProfiles.slowFast(
                this, HOME);
        return f.until(f::isDone)
                .withName("profileHomeTerminal");
    }

    public Command profileHomeToL1() {
        FollowJointProfiles f = FollowJointProfiles.fastSlow(
                this, L1);
        return f.until(f::isDone)
                .withName("profileHomeTerminal");
    }

    /**
     * Use a profile to move from the current position and velocity to the "home"
     * position (origin) at rest, and hold there forever.
     */
    public Command profileHomeEndless() {
        return FollowJointProfiles.slowFast(
                this, HOME)
                .withName("profileHomeEndless");
    }

    /**
     * Use a profile to move from the current position and velocity to the "home"
     * position at rest, and then turn off the elevator motors so they stop trying
     * to push against gravity (making that squealing noise).
     */
    public Command profileHomeAndThenRest() {
        MoveAndHold f = FollowJointProfiles.slowFast(this, HOME);
        return sequence(
                // f.until(f::isDone),
                f.withTimeout(2),
                restAtHome() //
        ).withName("profileHomeAndThenRest");

    }

    /** Turn off the elevator motor. */
    public Command restAtHome() {
        return run(this::rest);
    }

    /**
     * Use a profile to move from the current position and velocity to the floor at
     * rest, and stay there forever.
     */
    public Command pickWithProfile() {
        return FollowJointProfiles.fastSlow(
                this, CORAL_GROUND_PICK)
                .withName("pickWithProfile");
    }

    /**
     * Use a profile to move from the current position and velocity to the floor at
     * rest, and stay there forever.
     */
    public Command algaePickGround() {
        return FollowJointProfiles.algae(
                this, ALGAE_GROUND)
                .withName("pickWithProfile");
    }

    public FollowJointProfiles homeGentle() {
        return FollowJointProfiles.gentle(
                this, HOME);
    }

    public FollowJointProfiles homeAlgae() {
        return FollowJointProfiles.algaeUp(
                this, HOME);
    }

    public Command climbWithProfile() {
        return FollowJointProfiles.gentle(
                this, CLIMB)
                .withName("climbWithProfile");
    }

    /**
     * Use a profile to move from the current position and velocity to the
     * station-pick location at rest, and stay there forever.
     */
    public Command stationWithProfile() {
        return FollowJointProfiles.fastSlow(
                this, STATION)
                .withName("stationWithProfile");
    }

    /**
     * Use a profile to move from the current position and velocity to the
     * processor location at rest, and stay there forever.
     */
    public Command processorWithProfile() {
        return FollowJointProfiles.algae(
                this, PROCESSOR)
                .withName("processorWithProfile");
    }

    //////////////////////////////////////////////////////////////////
    ///
    /// TRAJECTORY COMMANDS
    ///

    public MoveAndHold homeToL1() {
        return m_transit.endless("homeToL1",
                HolonomicPose2d.make(m_home, -1.5),
                HolonomicPose2d.make(L2, -1.7));
    }

    // NEVER CALL
    public Command l1ToHome() {
        return m_transit.terminal("l1ToHome",
                HolonomicPose2d.make(L2, 1.3),
                HolonomicPose2d.make(m_home, 1.5));
    }

    public MoveAndHold homeToL2() {
        return m_transit.endless("homeToL2",
                HolonomicPose2d.make(m_home, 1.5),
                HolonomicPose2d.make(L2, 1.5));
    }

    public Command l2ToHome() {
        return m_transit.terminal("l2ToHome",
                HolonomicPose2d.make(L2, -1.5),
                HolonomicPose2d.make(m_home, -1.5));
    }

    public MoveAndHold homeToL3() {
        return m_transit.endless("homeToL3",
                HolonomicPose2d.make(m_home, 0.8),
                HolonomicPose2d.make(L3, 1.5));
    }

    public Command l3ToHome() {
        return m_transit.terminal("l3ToHome",
                HolonomicPose2d.make(L3, -1.5),
                HolonomicPose2d.make(m_home, -2.3));
    }

    public MoveAndHold homeToL4() {
        return m_transit.endless("homeToL4",
                HolonomicPose2d.make(m_home, 0.1),
                HolonomicPose2d.make(L4, 1.5));
    }

    public Command l4ToHome() {
        return m_transit.terminal("l4ToHome",
                HolonomicPose2d.make(L4, -1.5),
                HolonomicPose2d.make(m_home, -3));
    }

    public Command homeToAlgaeL2() {
        return m_transit.endless("homeToAlgaeL2",
                HolonomicPose2d.make(m_home, 1.5),
                HolonomicPose2d.make(ALGAE_L2, 1.5));
    }

    public Command homeToAlgaeL3() {
        return m_transit.endless("homeToAlgaeL3",
                HolonomicPose2d.make(m_home, 0),
                HolonomicPose2d.make(ALGAE_L3, 1.5));
    }

    public Command algaeL2ToHome() {
        return m_transit.terminal("homeToAlgaeL2",
                HolonomicPose2d.make(ALGAE_L2, -1.0),
                HolonomicPose2d.make(m_home, Math.PI));
    }

    public Command algaeL3ToHome() {
        return m_transit.terminal("homeToAlgaeL3",
                HolonomicPose2d.make(ALGAE_L3, -1.0),
                HolonomicPose2d.make(m_home, Math.PI));
    }

    /**
     * Move to the supplied point for algae pick from the reef, and hold there
     * forever.
     */
    public Command algaeReefPick(Supplier<ScoringLevel> level) {
        return select(Map.ofEntries(
                Map.entry(ScoringLevel.L3, homeToAlgaeL3()),
                Map.entry(ScoringLevel.L2, homeToAlgaeL2()) //
        ), level)
                .withName("algaeReefPick");
    }

    public Command algaeReefExit(Supplier<ScoringLevel> level) {
        return select(Map.ofEntries(
                Map.entry(ScoringLevel.L3, algaeL3ToHome()),
                Map.entry(ScoringLevel.L2, algaeL2ToHome()) //
        ), level)
                .withName("algaeReefExit");
    }

    /**
     * Move to the barge scoring position and hold there forever
     */
    public MoveAndHold homeToBarge() {
        return m_transit.endless("homeToBarge",
                HolonomicPose2d.make(m_home, 0),
                HolonomicPose2d.make(BARGE, -1));
    }

    public MoveAndHold bargeToHome() {
        return m_transit.endless("bargeToHome",
                HolonomicPose2d.make(BARGE, 2.5),
                HolonomicPose2d.make(m_home, Math.PI));

    }

    /** Not too far extended in any direction. */
    public boolean isSafeToDrive() {
        double x = m_elevatorBack.getPositionM();
        double y = m_shoulder.getWrappedPositionRad();
        return x < 1 && Math.abs(y) < 1;
    }

    /////////////////////////////////////////////////////////////

    public ElevatorArmWristKinematics getKinematics() {
        return m_kinematics;
    }

    public AnalyticalJacobian getJacobian() {
        return m_jacobian;
    }

    @Override
    public void periodic() {
        m_shoulder.periodic();
        m_elevatorFront.periodic();
        m_elevatorBack.periodic();
        m_wrist.periodic();
    }

    /////////////////////////////////////////////////////////////

    /** Elevator torque off, shoulder and wrist hold position at zero. */
    private void rest() {
        m_elevatorFront.stop();
        m_elevatorBack.stop();
        if (DISABLED) {
            m_wrist.setUnwrappedPosition(2, 0, 0, 0);
            return;
        }
        m_wrist.setUnwrappedPosition(0, 0, 0, 0);
        m_shoulder.setUnwrappedPosition(0, 0, 0, 0);
    }

    private void set(Config c, JointVelocities jv, JointAccelerations ja, JointForce jf) {
        logConfig(c, jv, ja, jf);
        m_elevatorFront.setPosition(c.shoulderHeight(), jv.elevator(), 0, jf.elevator());
        m_elevatorBack.setPosition(c.shoulderHeight(), jv.elevator(), 0, jf.elevator());
        if (DISABLED) {
            m_wrist.setUnwrappedPosition(2, 0, 0, 0);
            return;
        }
        m_wrist.setUnwrappedPosition(c.wristAngle(), jv.shoulder(), 0, jf.wrist());
        m_shoulder.setUnwrappedPosition(c.shoulderAngle(), jv.shoulder(), 0, jf.shoulder());
    }

    public Command setDisabled(boolean disabled) {
        return run(() -> {
            DISABLED = disabled;
        });
    }

    private void logConfig(Config c, JointVelocities jv, JointAccelerations ja, JointForce jf) {
        m_log_config.log(() -> c);
        m_log_jointV.log(() -> jv);
        m_log_jointA.log(() -> ja);
        m_log_jointF.log(() -> jf);
        Pose2d p = m_kinematics.forward(c);
        GlobalVelocityR3 v = m_jacobian.forward(c, jv);
        GlobalAccelerationR3 a = m_jacobian.forwardA(c, jv, ja);
        m_log_pose.log(() -> p);
        m_log_cartesianV.log(() -> v);
        m_log_cartesianA.log(() -> a);
    }

    /** to make the constants above easier to read */
    private static Rotation2d rad(double r) {
        return Rotation2d.fromRadians(r);
    };
}