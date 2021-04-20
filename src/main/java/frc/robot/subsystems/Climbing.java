
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbing extends SubsystemBase {
    private final Spark m_winch = new Spark(10);

    public void setWinch(double speed) {
      m_winch.set(speed);
    }
}