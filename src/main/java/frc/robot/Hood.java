package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood {
    private final Encoder m_hoodEncoder;
    private final DigitalInput m_hoodBack;
    private final DigitalInput m_hoodFront;
    private final SpeedController m_hoodMotor = new Spark(RobotConstants.getInstance().kHoodMotor);

    private boolean m_aiming = false; 
    private boolean m_zeroing = false;   
    public Hood() {
        if (!RobotConstants.kPractice) {
            m_hoodEncoder = new Encoder(4, 5);
            m_hoodBack = new DigitalInput(22);
            m_hoodFront = new DigitalInput(9);
        }
    }

    public boolean aim(double angle) {
        boolean aimed = false;

        double target = 0.2441*angle*angle*angle + 1.2016*angle*angle + 5.4673*angle + 667.17;
        SmartDashboard.putNumber("hood_target", target);

        if (m_hoodEncoder != null && m_hoodFront != null) {
            if (!m_aiming) {
                m_aiming = true;
                m_zeroing = true;
            }

            if (m_hoodEncoder.getRate() > 1700 || m_hoodEncoder.getRate() < -1700 || m_hoodEncoder.getDistance() > 2400 || m_hoodEncoder.getDistance() < -222) {
                m_zeroing = true;
            }

            if (m_hoodFront.get()) {
                m_zeroing = false;
                m_hoodEncoder.reset();
            }

            if (m_zeroing) {
                m_hoodMotor.set(-0.5);
            }
            else {
                if (m_hoodBack.get()) {
                    m_zeroing = true;
                    m_hoodMotor.set(0.0);
                }
                else if (m_hoodEncoder.getDistance() < target) {
                    m_hoodMotor.set(0.5);
                }
                else {
                    m_hoodMotor.set(0.0);
                    aimed = true;
                }
            }
        } else {
            m_hoodMotor.set(0.0);
        }

        SmartDashboard.putBoolean("hood_aimed", aimed);

        return aimed;
    }

    public void move(double speed) {
        m_aiming = false;

        if (m_hoodBack != null && m_hoodBack.get() && speed > 0) {
            m_hoodMotor.set(0.0);
        } else if (m_hoodFront != null && m_hoodFront.get() && speed < 0) {
            if (m_hoodEncoder != null) {
                m_hoodEncoder.reset();
            }
            m_hoodMotor.set(0.0);
        } else {
            m_hoodMotor.set(speed);
        }
    }
}
