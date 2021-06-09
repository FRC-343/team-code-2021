package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheel extends SubsystemBase{
    private final Spark m_wheel = new Spark(10);
    private final DoubleSolenoid m_wheelLift = new DoubleSolenoid(1, 6, 7);

    public void raise() {
        m_wheelLift.set(DoubleSolenoid.Value.kReverse);
    }

    public void lower() {
        m_wheelLift.set(DoubleSolenoid.Value.kForward);
    }

    public void raiseOrLower() {
        if (m_wheelLift.get() == DoubleSolenoid.Value.kReverse) {
            lower();
        } else {
            raise();
        }
    }

    public void setWheel(double speed) {
        m_wheel.set(speed);
    }


}
