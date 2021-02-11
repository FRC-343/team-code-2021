package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood {
    private final Encoder m_hoodEncoder;
    private final DigitalInput m_hoodZero;
    private final DigitalInput m_hoodForward;
    private final SpeedController m_hoodMotor = new Spark(RobotConstants.getInstance().kHoodMotor);

    private boolean m_aiming = false; 
    private boolean m_zeroing = false;   
    public Hood() {
        if (!RobotConstants.kPractice) {
            m_hoodEncoder = new Encoder(4, 5, true);
            m_hoodZero = new DigitalInput(22);
            m_hoodForward = new DigitalInput(9);
        }
    }

    public boolean aim(double angle) {
         boolean aimed = false;

         double target = -0.0357*angle*angle*angle - 7.3381*angle*angle - 74.092*angle + 3671.8 ; //60 + 2*angle - 0.208*angle*angle - 0.0287*angle*angle*angle;
         SmartDashboard.putNumber("hood_target", target);

         if (m_hoodEncoder != null && m_hoodZero != null) {
             if (!m_aiming) {
                 m_aiming = true;
                 m_zeroing = true;
             }

             if (m_hoodEncoder.getRate() > 1700 || m_hoodEncoder.getRate() < -1700 || m_hoodEncoder.getDistance() > 4400 || m_hoodEncoder.getDistance() < -222) {
                 m_zeroing = true;
             }

             if (m_hoodZero.get()) {
                 m_zeroing = false;
                 m_hoodEncoder.reset();
             }

             if (m_zeroing) {
                 m_hoodMotor.set(0.5);
             }
             else {
                 if (m_hoodForward.get()) {
                     m_zeroing = true;
                     m_hoodMotor.set(0.0);
                 }
                 else if (m_hoodEncoder.getDistance() < target) {
                     m_hoodMotor.set(-0.5);
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

        if (m_hoodZero != null && m_hoodZero.get() && speed > 0) {
            if (m_hoodEncoder != null) {
                m_hoodEncoder.reset();
            }
            m_hoodMotor.set(0.0);
        } else if (m_hoodForward != null && m_hoodForward.get() && speed < 0) {
            m_hoodMotor.set(0.0);
        } else {
            m_hoodMotor.set(speed);
        }
    }
}
