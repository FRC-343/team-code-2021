package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Vision;

public class AimCommand extends CommandBase {
    private final Vision m_vision;
    private final Hood m_hood;
    private final Drive m_drive;

    public AimCommand(Vision vision, Hood hooood, Drive drive) {
        m_vision = vision;
        m_hood = hooood;
        m_drive = drive;
        addRequirements(m_vision, m_hood, m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
