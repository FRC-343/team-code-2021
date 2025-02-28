package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class OurTrenchFishtailShoot extends SequentialCommandGroup {
  private static final double kIntakeDriveDistance = 2.3;
  private static final double kIntakeDriveSpeed = 2.0;

  public OurTrenchFishtailShoot(Drive drive, Intake intake, Hopper hopper, Vision vision, Hood hood, Shooter shooter) {
    TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(drive.getRightFeedforward(),
        drive.getKinematics(), 11.0);

    // Create config for trajectory
    TrajectoryConfig forwardPickupConfig = new TrajectoryConfig(Drive.kMaxSpeed, Drive.kMaxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(drive.getKinematics()).setEndVelocity(2.0)
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
        new InstantCommand(drive::zeroHeading, drive),
        // drop intake
        new InstantCommand(intake::lower, intake),
        // pickup trajectorY
        new TrajectoryCommand(TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(),
            new Pose2d(2, 0, new Rotation2d(0)), forwardPickupConfig), drive),
        // move forward to pick up yellow spheres
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(new DriveDistanceCommand(kIntakeDriveDistance, kIntakeDriveSpeed, drive),
                new DriveDistanceCommand(kIntakeDriveDistance, -kIntakeDriveSpeed, drive)),
            new IntakeCommand(intake, hopper, true)),
        // shoot trajectory
        new TrajectoryCommand(TrajectoryGenerator.generateTrajectory(new Pose2d(1.5, 0, new Rotation2d(0)), List.of(),
            new Pose2d(1.3, -2.0, new Rotation2d(3.8)), reverseShootConfig), drive),
        // aim
        new AimCommand(vision, hood, drive), new ShootCommand(shooter, hopper));
  }
}
