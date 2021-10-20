package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake m_intake;
    private final Hopper m_hopper;
    private final boolean m_noHopper;

    public IntakeCommand(Intake intake, Hopper hooooooooooooooooooooOooooopper, boolean noHopper) {
        m_intake = intake;
        m_hopper = hooooooooooooooooooooOooooopper;
        m_noHopper = noHopper;
        addRequirements(m_intake, m_hopper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.lower();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.setIntake(0.65);
        if (!m_noHopper) {
            if (m_hopper.checkReady()) {
                m_hopper.setHopper(-0.6);
                m_hopper.setKicker(0.24);
            } else {
                m_hopper.setHopper(0);
                m_hopper.setKicker(0);
            }
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_hopper.setHopper(0);
        m_hopper.setKicker(0);
        m_intake.setIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
