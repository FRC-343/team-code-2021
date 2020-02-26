package frc.robot;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

public class Autonomous {
        public final double kRamseteB = 2.0;
        public final double kRamseteZeta = 0.7;

        public enum State {
                INTAKE_DROP, MOVE_TO_PICKUP, INTAKE_FORWARD, INTAKE_BACKWARD, MOVE_TO_SHOOT, TARGET, SHOOT, END
        };

        private final double kIntakeTime = 0.6;
        private final double kShootTimeStart = 1.5;
        private final double kShootTimeEnd = 5.7;

        private final Timer m_timer = new Timer();

        private final Drive m_robotDrive;
        private final Hood m_aimer;
        private final Shooter m_shooter;
        private final SpeedController m_kicker;
        private final SpeedController m_hopper;
        private final SpeedController m_intake;
        private final DoubleSolenoid m_intakeLift;

        private final RamseteController m_follower;

        private DifferentialDriveWheelSpeeds m_prevSpeeds;
        private double m_prevTime;

        private Trajectory m_pickupTrajectory;
        private Trajectory m_shootTrajectory;

        private State m_state;

        public Autonomous(Drive robotDrive, Hood aimer, Shooter shooter, SpeedController kicker, SpeedController hopper,
                        SpeedController intake, DoubleSolenoid intakeLift) {
                m_robotDrive = robotDrive;
                m_aimer = aimer;
                m_shooter = shooter;
                m_kicker = kicker;
                m_hopper = hopper;
                m_intake = intake;
                m_intakeLift = intakeLift;

                TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                                m_robotDrive.getRightFeedforward(), m_robotDrive.getKinematics(), 10);

                // Create config for trajectory
                TrajectoryConfig forwardPickupConfig = new TrajectoryConfig(Drive.kMaxSpeed, Drive.kMaxAcceleration)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(m_robotDrive.getKinematics())
                                .setEndVelocity(0.7)
                                // Apply the voltage constraint
                                .addConstraint(voltageConstraint);

                // Create config for trajectory
                TrajectoryConfig reverseShootConfig = new TrajectoryConfig(Drive.kMaxSpeed, Drive.kMaxAcceleration)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(m_robotDrive.getKinematics())
                                // Apply the voltage constraint
                                .addConstraint(voltageConstraint).setReversed(true);

                // An example trajectory to follow. All units in meters.
                m_pickupTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(2.0, 0, new Rotation2d(0)),
                                // Pass config
                                forwardPickupConfig);

                m_shootTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(2.0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(0.2, 3.4999, new Rotation2d(135)),
                                // Pass config
                                reverseShootConfig);

                m_follower = new RamseteController(kRamseteB, kRamseteZeta);
        }

        public void autonomousInit() {
                m_prevTime = 0;

                m_prevSpeeds = new DifferentialDriveWheelSpeeds(0, 0);

                changeState(State.INTAKE_DROP);

                m_robotDrive.resetPID();
                m_robotDrive.resetOdometry(new Pose2d());
        }

        private void track(Trajectory trajectory) {
                double curTime = m_timer.get();
                double dt = curTime - m_prevTime;

                DifferentialDriveWheelSpeeds targetWheelSpeeds = m_robotDrive.getKinematics().toWheelSpeeds(
                                m_follower.calculate(m_robotDrive.getPose(), trajectory.sample(curTime)));

                double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
                double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

                double leftOutput;
                double rightOutput;

                double leftFeedforward = m_robotDrive.getLeftFeedforward().calculate(leftSpeedSetpoint,
                                (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

                double rightFeedforward = m_robotDrive.getRightFeedforward().calculate(rightSpeedSetpoint,
                                (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

                leftOutput = leftFeedforward + m_robotDrive.getLeftPIDController()
                                .calculate(m_robotDrive.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);

                rightOutput = rightFeedforward + m_robotDrive.getRightPIDController()
                                .calculate(m_robotDrive.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

                m_robotDrive.setVoltages(leftOutput, rightOutput);

                m_prevTime = curTime;
                m_prevSpeeds = targetWheelSpeeds;
        }

        private void changeState(State state) {
                m_state = state;

                m_timer.reset();
                m_timer.start();

                m_robotDrive.setVoltages(0, 0);
                m_intake.set(0.0);
                m_shooter.setVoltage(0.0);
                m_kicker.set(0.0);
                m_hopper.set(0.0);
        }

        public void autonomousPeriodic() {
                if (m_state == State.INTAKE_DROP) {
                        if (m_intakeLift != null) {
                                if (m_intakeLift.get() != Value.kForward) {
                                        m_intakeLift.set(Value.kForward);
                                } else {
                                        m_intakeLift.set(Value.kOff);
                                        changeState(State.MOVE_TO_PICKUP);
                                }
                        } else {
                                changeState(State.MOVE_TO_PICKUP);
                        }
                } else if (m_state == State.MOVE_TO_PICKUP) {
                        track(m_pickupTrajectory);

                        if (m_prevTime > m_pickupTrajectory.getTotalTimeSeconds()) {
                                changeState(State.INTAKE_FORWARD);
                        }
                } else if (m_state == State.INTAKE_FORWARD) {
                        m_intake.set(0.65);
                        m_robotDrive.drive(0.7, 0.0);

                        if (m_timer.get() > kIntakeTime) {
                                changeState(State.INTAKE_BACKWARD);
                        }
                } else if (m_state == State.INTAKE_BACKWARD) {
                        m_intake.set(0.65);
                        m_robotDrive.drive(-0.7, 0.0);

                        if (m_timer.get() > kIntakeTime) {
                                changeState(State.MOVE_TO_SHOOT);
                        }
                } else if (m_state == State.MOVE_TO_SHOOT) {
                        track(m_shootTrajectory);

                        if (m_prevTime > m_shootTrajectory.getTotalTimeSeconds()) {
                                changeState(State.TARGET);
                        }
                } else if (m_state == State.TARGET) {
                        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
                        NetworkTableEntry tx = table.getEntry("tx");
                        NetworkTableEntry ty = table.getEntry("ty");

                        double driveCommand = 0.0;
                        double steerCommand = 0.0;

                        double heading_error = -tx.getDouble(0.0);
                        double angle_error = ty.getDouble(0.0);

                        boolean steerReady = false;
                        boolean aimReady = false;

                        if (heading_error > 1.0) {
                                steerCommand = Robot.kTargetP * heading_error + Robot.kMinTargetCommand;
                        } else if (heading_error < -1.0) {
                                steerCommand = Robot.kTargetP * heading_error - Robot.kMinTargetCommand;
                        } else if (m_timer.get() > 1.5) {
                                steerReady = true;
                        }

                        aimReady = m_aimer.aim(angle_error);

                        m_robotDrive.drive(driveCommand, steerCommand);

                        if (steerReady && aimReady) {
                                changeState(State.SHOOT);
                        }
                } else if (m_state == State.SHOOT) {
                        m_shooter.shoot(Shooter.kShootSpeed);

                        if (m_timer.get() > kShootTimeStart && m_shooter.getRate() > Shooter.kShootReadySpeed) {
                                m_kicker.set(-1);
                                m_hopper.set(0.65);
                        } else {
                                m_kicker.set(0);
                                m_hopper.set(0);
                        }

                        if (m_timer.get() > kShootTimeEnd) {
                                changeState(State.END);
                        }
                }

        }

        public void autonomousEnd() {
                m_timer.stop();
                m_robotDrive.setVoltages(0, 0);
        }
}