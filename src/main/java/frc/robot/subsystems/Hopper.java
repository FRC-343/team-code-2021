package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Debouncer;

public class Hopper extends SubsystemBase {
    private final Spark m_hopper = new Spark(9);
    private final Spark m_kicker = new Spark(4);

    private final DigitalInput m_cellDetector = new DigitalInput(8);
    private final Debouncer m_cellDetectorDebouncer = new Debouncer();

    public Hopper() {
        SendableRegistry.setSubsystem(m_hopper, this.getClass().getSimpleName());
        SendableRegistry.setName(m_hopper, "Hopper Motor");

        SendableRegistry.setSubsystem(m_kicker, this.getClass().getSimpleName());
        SendableRegistry.setName(m_kicker, "Kicker Motor");

        SendableRegistry.setSubsystem(m_cellDetector, this.getClass().getSimpleName());
        SendableRegistry.setName(m_cellDetector, "Hopper Cell Detector");
    }

    public boolean checkReady() {
        return m_cellDetectorDebouncer.isReady(m_cellDetector.get()) && !m_cellDetector.get();
    }

    public void setHopper(double speed) {
        m_hopper.set(speed);
    }

    public void setKicker(double speed) {
        m_kicker.set(speed);
    }
}
