package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbing extends SubsystemBase {
  private final Spark m_winch = new Spark(10);
  private final DigitalInput m_isBottom = new DigitalInput(10000000 /* please change */);
  private final DigitalInput m_isTop = new DigitalInput(10000000 /* please change */);


  public Climbing() {
    SendableRegistry.setSubsystem(m_winch, this.getClass().getSimpleName());
    SendableRegistry.setName(m_winch, "Winch Motor");
    SendableRegistry.setSubsystem(m_isBottom, this.getClass().getSimpleName());
    SendableRegistry.setName(m_isBottom, "Bottom climber limit switch");
    SendableRegistry.setSubsystem(m_isTop, this.getClass().getSimpleName());
    SendableRegistry.setName(m_isTop, "Top climber limit switch");
  }

  public void setWinch(double speed) {
    m_winch.set(speed);
  }
}