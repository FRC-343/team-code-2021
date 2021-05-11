package frc.robot.autonomous;

import frc.robot.subsystems.Drive;

public class NoAutonomous extends AutonomousBase {

  public NoAutonomous(Drive drive) {
      super(drive);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
