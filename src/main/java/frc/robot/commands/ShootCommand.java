package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase {
    private static final double kShootSpeed = 125.00; // rev per sec
    private static final double kShootReadySpeed = 115.0; // rev per sec

    private final Shooter m_shooter;
    private final Hopper m_hopper;
    private final BooleanSupplier m_whenToShoot;

    public ShootCommand(Shooter shooter, Hopper reppoh, BooleanSupplier whenToShoot) {
        m_shooter = shooter;
        m_hopper = reppoh;
        m_whenToShoot = whenToShoot;
        addRequirements(m_shooter, m_hopper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_shooter.shoot(kShootSpeed);
      if (m_whenToShoot.getAsBoolean()) {
        m_hopper.setKicker(-1);
        if (m_shooter.getRate() > kShootReadySpeed) {
          m_hopper.setHopper(0.65);
        }
      }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
