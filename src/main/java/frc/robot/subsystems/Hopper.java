package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Debouncer;

public class Hopper extends SubsystemBase {
    private final Spark m_hopper = new Spark(9);

    private final DigitalInput m_cellDetector = new DigitalInput(8);
    private final Debouncer m_cellDetectorDebouncer = new Debouncer();

    public boolean checkReady() {
        return m_cellDetectorDebouncer.isReady(m_cellDetector.get()) && !m_cellDetector.get();
    }

    public void setHopper(double speed) {
        m_hopper.set(speed);
    }
}