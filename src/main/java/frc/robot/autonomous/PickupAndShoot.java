package edu.wpi.first.wpilibj.examples.hatchbottraditional.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.Drive;

public class PickupAndShoot extends SequentialCommandGroup {
  private static final double kIntakeDriveDistance = 0.42;
  private static final double kIntakeDriveSpeed = 0.7;

  public PickupAndShoot(Drive drive, Intake intake, Hopper hopper, Vision vision, Hood hood) {
    TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
        drive.getRightFeedforward(), drive.getKinematics(), 11.0);

    // Create config for trajectory
    TrajectoryConfig forwardPickupConfig = new TrajectoryConfig(Drive.kMaxSpeed, Drive.kMaxAcceleration)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(drive.getKinematics()).setEndVelocity(0.7)
      // Apply the voltage constraint
      .addConstraint(voltageConstraint);

    // Create config for trajectory
    TrajectoryConfig reverseShootConfig = new TrajectoryConfig(Drive.kMaxSpeed, Drive.kMaxAcceleration)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(drive.getKinematics())
      // Apply the voltage constraint
      .addConstraint(voltageConstraint).setReversed(true);

    // commands in this autonomous
    addCommands(
        // drop intake
        new InstantCommand(intake::lower, intake),
        // pickup trajectory
        new TrajectoryCommand(TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2.0, 0, new Rotation2d(0)),
            // Pass config
            forwardPickupConfig), drive),
        // move forward to pick up yellow spheres
        new ParallelDeadlineGroup(
          new SequentialCommandGroup(
            new DriveDistanceCommand(kIntakeDriveDistance, kIntakeDriveSpeed, drive),
            new DriveDistanceCommand(kIntakeDriveDistance, -kIntakeDriveSpeed, drive),
            ),
          new IntakeCommand(intake, hopper),
          ),
        // shoot trajectory
        new TrajectoryCommand(TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(2.0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(0.2, 3.4999, new Rotation2d(135)),
              // Pass config
              reverseShootConfig), drive),
        // aim
        new AimCommand(vision, hood, drive),
        new ShootCommand(shooter, hopper),
        );
  }
}
