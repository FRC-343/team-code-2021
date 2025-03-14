package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbing extends SubsystemBase {
  private final Spark m_winch = new Spark(11);
  private final DigitalInput m_isBottom = new DigitalInput(12);
  private final DigitalInput m_isTop = new DigitalInput(13);
  private final DoubleSolenoid m_ratchetLift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  public Climbing() {
    m_winch.setInverted(true);
    SendableRegistry.setSubsystem(m_winch, this.getClass().getSimpleName());
    SendableRegistry.setName(m_winch, "Winch Motor");
    SendableRegistry.setSubsystem(m_isBottom, this.getClass().getSimpleName());
    SendableRegistry.setName(m_isBottom, "Bottom climber limit switch");
    SendableRegistry.setSubsystem(m_isTop, this.getClass().getSimpleName());
    SendableRegistry.setName(m_isTop, "Top climber limit switch");
    SendableRegistry.setSubsystem(m_ratchetLift, this.getClass().getSimpleName());
    SendableRegistry.setName(m_ratchetLift, "Ratchet for climbing witch");
  }

  public void setWinch(double speed) {
    if (m_isBottom.get() || m_ratchetLift.get() == DoubleSolenoid.Value.kForward) {
      if (speed < 0) {
        m_winch.set(0.0);
      } else {
        m_winch.set(speed);
      }
    } else if (m_isTop.get()) {
      if (speed > 0) {
        m_winch.set(0.0);
      } else {
        m_winch.set(speed);
      }
    } else {
      m_winch.set(speed);
    }

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Ratchet State", m_ratchetLift.get() == DoubleSolenoid.Value.kForward);
  }

  public void disEngage() {
    m_ratchetLift.set(DoubleSolenoid.Value.kReverse);
  }

  public void engage() {
    m_ratchetLift.set(DoubleSolenoid.Value.kForward);
  }

  public void toBeOrNotToBe() {
    if (m_ratchetLift.get() == DoubleSolenoid.Value.kReverse) {
      engage();
    } else {
      disEngage();
    }
  }
}