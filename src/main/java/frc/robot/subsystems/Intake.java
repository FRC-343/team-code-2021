
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final DoubleSolenoid m_intakeLift = new DoubleSolenoid(1, 0, 1);
    private final Spark m_intake = new Spark(7);

    public Intake() {
        m_intake.setInverted(true);
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