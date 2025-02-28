package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    private final Encoder m_hoodEncoder = new Encoder(4, 5);
    private final DigitalInput m_hoodBack = new DigitalInput(22);
    private final DigitalInput m_hoodFront = new DigitalInput(9);
    private final Spark m_hoodMotor = new Spark(6);

    private boolean m_aimed = false; // if shooter is currently aimed
    private double m_target = 0.0; // where it needs to be aiming
    private double m_speed = 0.0; // manual control
    private boolean m_aiming = false; // if currently aiming (for automatic)
    private boolean m_zeroing = false; // resetting hood

    public Hood() {
        SendableRegistry.setSubsystem(m_hoodEncoder, this.getClass().getSimpleName());
        SendableRegistry.setName(m_hoodEncoder, "Hood Encoder");
        SendableRegistry.setSubsystem(m_hoodBack, this.getClass().getSimpleName());
        SendableRegistry.setName(m_hoodBack, "Hood Back Limit");
        SendableRegistry.setSubsystem(m_hoodBack, this.getClass().getSimpleName());
        SendableRegistry.setName(m_hoodFront, "Hood Front Limit");
        SendableRegistry.setSubsystem(m_hoodMotor, this.getClass().getSimpleName());
        SendableRegistry.setName(m_hoodMotor, "Hood Motor");

    }

    public void aim(double angle) {
        m_target = 4.2425 * angle * angle + 142.56 * angle + 1491.1;
        SmartDashboard.putNumber("hood_target", m_target);

        if (!m_aiming) {
            m_aiming = true;
            m_zeroing = true;
        }
    }

    public boolean isAimed() {
        return m_aimed;
    }

    public void move(double speed) {
        m_aiming = false;
        m_aimed = false;
        m_speed = speed;
    }

    @Override
    public void periodic() {
        if (m_aiming) {

            if (m_hoodEncoder.getRate() > 1700 || m_hoodEncoder.getRate() < -1700 || m_hoodEncoder.getDistance() > 4500
                    || m_hoodEncoder.getDistance() < -222) {
                System.err.println("Hood encoder sent garbage values, zeroing again...");
                m_zeroing = true;
            }

            if (m_hoodFront.get()) {
                m_zeroing = false;
                m_hoodEncoder.reset();
            }

            if (m_zeroing) {
                m_hoodMotor.set(-0.5);
            } else {
                if (m_hoodBack.get()) {
                    m_zeroing = true;
                    m_hoodMotor.set(0.0);
                } else if (m_hoodEncoder.getDistance() < m_target) {
                    m_hoodMotor.set(0.5);
                } else {
                    m_hoodMotor.set(0.0);
                    m_aimed = true;
                }
            }
            SmartDashboard.putBoolean("hood_aimed", m_aimed);
        } else {
            if (m_hoodBack.get() && m_speed > 0.0) {
                m_hoodMotor.set(0.0);
            } else if (m_hoodFront.get() && m_speed < 0.0) {
                m_hoodEncoder.reset();
                m_hoodMotor.set(0.0);
            } else {
                m_hoodMotor.set(m_speed);
            }

        }
    }
}
