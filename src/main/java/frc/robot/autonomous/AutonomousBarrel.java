package frc.robot.autonomous;

import frc.robot.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

public class AutonomousBarrel extends Autonomous {
    Trajectory m_trajectory = new Trajectory();

    public AutonomousBarrel(Drive robotDrive, Hood aimer, Shooter shooter, SpeedController kicker,
            SpeedController hopper, SpeedController intake, DoubleSolenoid intakeLift) {
        super(robotDrive, aimer, shooter, kicker, hopper, intake, intakeLift);
        String trajectoryJSON = "paths/BarrelRace.wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }

    public void autonomousPeriodic() {
        boolean running = false;

        if (m_state == "start") {
            changeState("first");
        } else if (m_state == "first") {
            running = track(m_trajectory);
            if (!running) {
                changeState("end");
            }
        }
    }
}