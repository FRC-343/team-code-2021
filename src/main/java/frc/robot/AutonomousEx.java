package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

public class AutonomousEx extends Autonomous {
    private static final double kIntakeTime = 0.6;

    private Trajectory m_pickupTrajectory;
    private Trajectory m_shootTrajectory;

    public AutonomousEx(Drive robotDrive, Hood aimer, Shooter shooter, SpeedController kicker, SpeedController hopper,
            SpeedController intake, DoubleSolenoid intakeLift) {
        super(robotDrive, aimer, shooter, kicker, hopper, intake, intakeLift);

        TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                m_robotDrive.getRightFeedforward(), m_robotDrive.getKinematics(), 10);

        // Create config for trajectory
        TrajectoryConfig forwardPickupConfig = new TrajectoryConfig(Drive.kMaxSpeed, Drive.kMaxAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(m_robotDrive.getKinematics()).setEndVelocity(0.7)
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
    }

    public void autonomousPeriodic() {
        boolean running = false;

        if (m_state == "intake_drop") {
            running = dropIntake();

            if (!running) {
                changeState("move_to_pickup");
            }
        } else if (m_state == "move_to_pickup") {
            running = track(m_pickupTrajectory);

            if (!running) {
                changeState("intake_forward");
            }
        } else if (m_state == "intake_forward") {
            m_intake.set(0.65);
            m_robotDrive.drive(0.7, 0.0);

            if (m_timer.get() > kIntakeTime) {
                changeState("intake_backward");
            }
        } else if (m_state == "intake_backward") {
            m_intake.set(0.65);
            m_robotDrive.drive(-0.7, 0.0);

            if (m_timer.get() > kIntakeTime) {
                changeState("move_to_shoot");
            }
        } else if (m_state == "move_to_shoot") {
            running = track(m_shootTrajectory);

            if (!running) {
                changeState("target");
            }
        } else if (m_state == "target") {
            running = target();

            if (!running) {
                changeState("shoot");
            }
        } else if (m_state == "shoot") {
            running = shoot();

            if (!running) {
                changeState("end");
            }
        }
    }
}