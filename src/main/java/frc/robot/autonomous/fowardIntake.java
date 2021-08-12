package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.DriveDistanceCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class fowardIntake extends SequentialCommandGroup {
  private static final double kBackupDriveDistance = 1;
  private static final double kBackupDriveSpeed = 1;

  public fowardIntake(Drive drive, Intake intake) {
    // commands in this autonomous
    addCommands(
  
        new DriveDistanceCommand(kBackupDriveDistance, kBackupDriveSpeed, drive));
     
 
      
  }
}
