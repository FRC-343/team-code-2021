package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wheel extends SubsystemBase {
    private final Spark m_wheel = new Spark(10);
    private final DoubleSolenoid m_wheelLift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);


    private static final Color kRed = new Color(0.518311, 0.344971, 0.136963);
    private static final Color kGreen = new Color(0.1689, 0.575439, 0.25585);
    private static final Color kBlue = new Color(0.1267, 0.4160, 0.4575);
    private static final Color kYellow = new Color(0.320068, 0.558105, 0.122070);

    public Wheel() {


        SendableRegistry.setSubsystem(m_wheel, this.getClass().getSimpleName());
        SendableRegistry.setName(m_wheel, "Wheel Spinner Motor");

        SendableRegistry.setSubsystem(m_wheelLift, this.getClass().getSimpleName());
        SendableRegistry.setName(m_wheelLift, "Wheel Spinner Lift");
    }

    @Override
    public void periodic() {
    }

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
