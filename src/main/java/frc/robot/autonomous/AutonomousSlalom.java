package frc.robot.autonomous;

import frc.robot.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

public class AutonomousSlalom extends Autonomous {
    Trajectory m_trajectory = new Trajectory();

    public AutonomousSlalom(Drive robotDrive, Hood aimer, Shooter shooter, SpeedController kicker,
            SpeedController hopper, SpeedController intake, DoubleSolenoid intakeLift) {
        super(robotDrive, aimer, shooter, kicker, hopper, intake, intakeLift);
        TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                m_robotDrive.getRightFeedforward(), m_robotDrive.getKinematics(), 11.0);

        TrajectoryConfig forwardConfig = new TrajectoryConfig(0.1 * Drive.kMaxSpeed, Drive.kMaxAcceleration)
                .setKinematics(m_robotDrive.getKinematics()).addConstraint(voltageConstraint);

        // All units in meters except the ones in radians (I think)
        // starts facing positive x by default not positive y

        // new Translation2d(x, y)
        m_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(2 * Math.PI)),
                List.of(new Translation2d(1.52, 0.762),
                new Translation2d(3.05, 1.52),
                new Translation2d(4.57, 1.52),
                new Translation2d(6.1, 0.762),
                new Translation2d(6.86, 0.0),
                new Translation2d(7.62, 0.762),
                new Translation2d(6.86, 1.52),
                new Translation2d(6.1, 0.762),
                new Translation2d(4.57, 0.0),
                new Translation2d(3.05, 0.0),
                new Translation2d(1.52, 0.762)), 
                new Pose2d(0.381, 1.52, new Rotation2d(-Math.PI)), forwardConfig);
    }

    public void autonomousPeriodic() {
        boolean running = false;

        if (m_state == "start") {
            changeState("first");
        } else if (m_state == "first") {
            running = track(m_trajectory);
            if (!running) {
                changeState("end");
            }
        }
    }
}