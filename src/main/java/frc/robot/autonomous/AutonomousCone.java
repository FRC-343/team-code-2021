package frc.robot.autonomous;

import frc.robot.*;

import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

public class AutonomousCone extends Autonomous {
    private static final double kIntakeTime = 0.6;

    private Trajectory m_firstTrajectory;
    private Trajectory m_secondTrajectory;
    private Trajectory m_thirdTrajectory;
    private Trajectory m_fourthTrajectory;

    public AutonomousCone(Drive robotDrive, Hood aimer, Shooter shooter, SpeedController kicker, SpeedController hopper,
            SpeedController intake, DoubleSolenoid intakeLift) {
        super(robotDrive, aimer, shooter, kicker, hopper, intake, intakeLift);

        TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                m_robotDrive.getRightFeedforward(), m_robotDrive.getKinematics(), 11.0);

        TrajectoryConfig forwardConfig = new TrajectoryConfig(0.3*Drive.kMaxSpeed, Drive.kMaxAcceleration)
                .setKinematics(m_robotDrive.getKinematics()).addConstraint(voltageConstraint);

        TrajectoryConfig reverseConfig = new TrajectoryConfig(0.3*Drive.kMaxSpeed, Drive.kMaxAcceleration)
                .setKinematics(m_robotDrive.getKinematics()).addConstraint(voltageConstraint).setReversed(true);

        // All units in meters except the ones in radians (I think) starts facing
        // positive x by default not positive y
        m_firstTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(),
                new Pose2d(1.524, 1.001, new Rotation2d((Math.PI / 2))), forwardConfig);
        /* new Translation2d(x, y) */
        m_secondTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.524, 1.001, new Rotation2d((Math.PI / 2))),
                List.of(new Translation2d(3.23, -1.559910000003435), new Translation2d(3.792, -0.579)),
                new Pose2d(3.792, 1.001, new Rotation2d(-(Math.PI / 2))), reverseConfig);

        m_thirdTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(3.792, 1.001, new Rotation2d(-(Math.PI / 2))), 
        List.of(new Translation2d(4.091, -1.524), new Translation2d(4.953, -1.654), new Translation2d(5.715, -1.524)), new Pose2d(6.178, 1.001, new Rotation2d(((Math.PI / 2)))), forwardConfig);

        m_fourthTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(6.078, 1.001, new Rotation2d(((Math.PI / 2)))), List.of(), new Pose2d(7.40, 0.0, new Rotation2d(Math.PI)),
        reverseConfig);
    }

    public void autonomousPeriodic() {
        boolean running = false;

        if (m_state == "start") {
            changeState("first");
        } else if (m_state == "first") {
            running = track(m_firstTrajectory);
            if (!running) {
                changeState("second");
            }
        } else if (m_state == "second") {
            running = track(m_secondTrajectory);
            if (!running) {
                changeState("third");
            }
        } else if (m_state == "third") {
            running = track(m_thirdTrajectory);
            if (!running) {
                changeState("fourth");
            }
        } else if (m_state == "fourth") {
            running = track(m_fourthTrajectory);
            if (!running) {
                changeState("end");
            }
        }

        // if (m_state == "start") {
        // changeState("intake_drop");
        // } else if (m_state == "intake_drop") {
        // running = dropIntake();

        // if (!running) {
        // changeState("move_to_pickup");
        // }
        // } else if (m_state == "move_to_pickup") {
        // running = track(m_pickupTrajectory);

        // if (!running) {
        // changeState("intake_forward");
        // }
        // } else if (m_state == "intake_forward") {
        // m_intake.set(0.65);
        // m_robotDrive.drive(0.7, 0.0);

        // if (m_timer.get() > kIntakeTime) {
        // changeState("intake_backward");
        // }
        // } else if (m_state == "intake_backward") {
        // m_intake.set(0.65);
        // m_robotDrive.drive(-0.7, 0.0);

        // if (m_timer.get() > kIntakeTime) {
        // changeState("move_to_shoot");
        // }
        // } else if (m_state == "move_to_shoot") {
        // m_intake.set(0.65);
        // running = track(m_shootTrajectory);

        // if (!running) {
        // changeState("target");
        // }
        // } else if (m_state == "target") {
        // running = target();

        // if (!running) {
        // changeState("shoot");
        // }
        // } else if (m_state == "shoot") {
        // running = shoot();

        // if (!running) {
        // changeState("end");
        // }
        // }
    }
}