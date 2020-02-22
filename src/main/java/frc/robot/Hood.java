package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;

public class Hood {
    private final Encoder m_hoodEncoder;
    private final DigitalInput m_hoodZero;
    private final SpeedController m_hoodMotor = new Spark(RobotConstants.getInstance().kHoodMotor);

    public Hood() {
        if (!RobotConstants.kPractice) {
            m_hoodEncoder = new Encoder(4, 5);
            m_hoodZero = new DigitalInput(9);
        }
    }

    public void aim(double height) {
        // m_hoodEncoder.get( /*aiming code*/ );

    }

    public void move(double speed) {
        if (m_hoodZero != null && m_hoodZero.get() && speed < 0) {
            m_hoodEncoder.reset();
            m_hoodMotor.set(0.0);
        } else {
            m_hoodMotor.set(speed);
        }
    }
}
