package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Shooter {
    public static final double kShootSpeed = 0.8; // ratio
    public static final double kShootReadySpeed = 105.0; // rev per sec

    private final SpeedController m_shooter = new Spark(5);

    private final Encoder m_shooterEncoder;
    
    private final PIDController m_shooterPIDController = new PIDController(0.153, 0.0, 0.0);
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(1.71, 0.0782);

    public Shooter() {
        m_shooter.setInverted(true);

        if (!RobotConstants.kPractice) {
            m_shooterEncoder = new Encoder(6, 7);
            m_shooterEncoder.setDistancePerPulse(0.333);
            m_shooterEncoder.setReverseDirection(true);
        }
    }

    public double getRate() {
        return m_shooterEncoder.getRate();
    }

    public void setVoltage(double volts) {
        m_shooter.setVoltage(volts);
    }

    public void shoot(double speed) {
        double shooterFeedforward = m_shooterFeedforward.calculate(speed);
        double shooterPIDOutput = 0.0;
        if (m_shooterEncoder != null) {
            shooterPIDOutput = m_shooterPIDController.calculate(m_shooterEncoder.getRate(), speed);
        }

        double shooterOutput = shooterFeedforward + shooterPIDOutput;
        
        m_shooter.setVoltage(shooterOutput);
    }
}