package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableRegistry;

public class Shooter extends SubsystemBase {
    private static final double kShootGarbage = 150.0; // rev per sec, for irregular values

    private final Spark m_shooter = new Spark(5);

    private final Encoder m_shooterEncoder = new Encoder(6, 7);

    private final PIDController m_shooterPIDController = new PIDController(0.10, 0.0, 0.0);
    private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(1.71, 0.0782);

    private double m_speed = 0.0;

    private double m_lastRate = 0.0;

    public Shooter() {
        m_shooter.setInverted(true);
        m_shooterEncoder.setDistancePerPulse(0.333);
        m_shooterEncoder.setReverseDirection(true);

        SendableRegistry.setSubsystem(m_shooterEncoder, this.getClass().getSimpleName());
        SendableRegistry.setName(m_shooterEncoder, "Shooter Encoder");

        SendableRegistry.setSubsystem(m_shooterPIDController, this.getClass().getSimpleName());
        SendableRegistry.setName(m_shooterPIDController, "Shooter PIDController");

        SendableRegistry.setSubsystem(m_shooter, this.getClass().getSimpleName());
        SendableRegistry.setName(m_shooter, "Shooter Motor");
    }

    public double getRate() {
        if (m_shooterEncoder.getRate() < kShootGarbage) {
            m_lastRate = m_shooterEncoder.getRate();
        }

        return m_lastRate;
    }

    public void shoot(double speed) {
        m_speed = speed;
    }

    @Override
    public void periodic() {
        if (m_speed > 0.01) {
            double shooterFeedforward = m_shooterFeedforward.calculate(m_speed);
            double shooterPIDOutput = m_shooterPIDController.calculate(getRate(), m_speed);
            double shooterOutput = shooterFeedforward + shooterPIDOutput;

            m_shooter.setVoltage(shooterOutput);
        } else {
            m_shooter.setVoltage(0.0);
        }
    }
}
