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

public class AutonomousSimple extends Autonomous {
    private Trajectory m_backupTrajectory;

    public AutonomousSimple(Drive robotDrive, Hood aimer, Shooter shooter, SpeedController kicker,
            SpeedController hopper, SpeedController intake, DoubleSolenoid intakeLift) {
        super(robotDrive, aimer, shooter, kicker, hopper, intake, intakeLift);

        TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                m_robotDrive.getRightFeedforward(), m_robotDrive.getKinematics(), 10);


        // Create config for trajectory
        TrajectoryConfig reverseBackupConfig = new TrajectoryConfig(Drive.kMaxSpeed, Drive.kMaxAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(m_robotDrive.getKinematics())
                // Apply the voltage constraint
                .addConstraint(voltageConstraint).setReversed(true);

                m_backupTrajectory=TrajectoryGenerator.generateTrajectory(
                    // Start at the origin facing the +X direction
                    new Pose2d(0,0,new Rotation2d(0)),
                    // Pass through these two interior waypoints, making an 's' curve path
                    List.of(),
                    // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(-1,0,new Rotation2d(0)),
                    // Pass config
                    reverseBackupConfig);
    }

    public void autonomousPeriodic() {
        boolean running = false;

        if (m_state == "target") {
            running = target();

            if (!running) {
                changeState("shoot");
            }
        } else if (m_state == "shoot") {
            running = shoot();

            if (!running) {
                changeState("backup");
            }
        } else if (m_state == "backup") {
            running = track(m_backupTrajectory);

            if (!running) {
                changeState("end");
            }
        }
    }
}