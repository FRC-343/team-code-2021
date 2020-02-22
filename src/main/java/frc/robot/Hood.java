package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;

public class Hood {
    private final Encoder m_hoodEncoder;
    private final SpeedController m_hoodMotor = new Spark(RobotConstants.getInstance().kHoodMotor);

    public Hood() {
        if (!RobotConstants.kPractice) {
            m_hoodEncoder = new Encoder(4, 5);
            m_hoodMotor.setInverted(true);
        }
    }

    public void aim(double height) {
        //m_hoodEncoder.get( /*aiming code*/ );
    }
    public void move(double speed) {
        m_hoodMotor.set(speed);
    }
}
