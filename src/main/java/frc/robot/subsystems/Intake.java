
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final DoubleSolenoid m_intakeLift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private final Spark m_intake = new Spark(7);

    public Intake() {
        m_intake.setInverted(true);

        SendableRegistry.setSubsystem(m_intake, this.getClass().getSimpleName());
        SendableRegistry.setName(m_intake, "Intake Motor");

        SendableRegistry.setSubsystem(m_intakeLift, this.getClass().getSimpleName());
        SendableRegistry.setName(m_intakeLift, "Intake Lift");
    }

    public void raise() {
        m_intakeLift.set(DoubleSolenoid.Value.kReverse);
    }

    public void lower() {
        m_intakeLift.set(DoubleSolenoid.Value.kForward);
    }

    public void setIntake(double speed) {
        m_intake.set(speed);
    }
}
