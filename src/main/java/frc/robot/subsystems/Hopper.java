package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Hopper extends SubsystemBase {
    private final Spark m_hopper = new Spark(RobotConstants.getInstance().kHopper);

    public void setHopper(double speed) {
        m_hopper.set(speed);
    }
}