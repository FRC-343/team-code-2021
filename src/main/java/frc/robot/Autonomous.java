package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

public class Autonomous {
    public final double kRamseteB = 2.0;
    public final double kRamseteZeta = 0.7;

    private final Timer m_timer = new Timer();

    private final Drive m_robotDrive;
    private final RamseteController m_follower;

    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    private Trajectory m_trajectory;

    public Autonomous(Drive robotDrive) {
        m_robotDrive = robotDrive;

        TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                m_robotDrive.getRightFeedforward(),
                m_robotDrive.getKinematics(), 5);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Drive.kMaxSpeed, Drive.kMaxAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(m_robotDrive.getKinematics())
                // Apply the voltage constraint
                .addConstraint(voltageConstraint);

        // An example trajectory to follow. All units in meters.
        m_trajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1.0, 1.0), new Translation2d(2.0, -1.0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3.0, 0, new Rotation2d(0)),
                // Pass config
                config);

        m_follower = new RamseteController(kRamseteB, kRamseteZeta);
    }

    public void autonomousInit() {
        m_prevTime = 0;
        Trajectory.State initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_robotDrive.getKinematics().toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));

        m_timer.reset();
        m_timer.start();

        m_robotDrive.resetPID();
        m_robotDrive.resetOdometry(new Pose2d());
    }

    public void autonomousPeriodic() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        DifferentialDriveWheelSpeeds targetWheelSpeeds = m_robotDrive.getKinematics()
                .toWheelSpeeds(m_follower.calculate(m_robotDrive.getPose(), m_trajectory.sample(curTime)));

        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

            double leftFeedforward = m_robotDrive.getLeftFeedforward().calculate(leftSpeedSetpoint,
                    (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward = m_robotDrive.getRightFeedforward().calculate(rightSpeedSetpoint,
                    (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

            leftOutput = leftFeedforward
                    + m_robotDrive.getLeftPIDController().calculate(m_robotDrive.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);

            rightOutput = rightFeedforward
                    + m_robotDrive.getRightPIDController().calculate(m_robotDrive.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

        m_robotDrive.setVoltages(leftOutput, rightOutput);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;
    }

    public void autonomousEnd() {
        m_timer.stop();
        m_robotDrive.setVoltages(0, 0);
    }
}