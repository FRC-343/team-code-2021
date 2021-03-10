package frc.robot.autonomous;

import frc.robot.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.AnalogInput;
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

public class AutonomousGalactic extends Autonomous {
    private final AnalogInput m_greg;

    private final Trajectory m_trajectory;

    private enum Selection { NONE, A_RED, A_BLUE, B_RED, B_BLUE };
    private Selection m_selection;

    public AutonomousGalactic(Drive robotDrive, Hood aimer, Shooter shooter, SpeedController kicker,
            SpeedController hopper, SpeedController intake, DoubleSolenoid intakeLift, AnalogInput greg) {
        super(robotDrive, aimer, shooter, kicker, hopper, intake, intakeLift);

        m_greg = greg;

        m_selection = NONE;

        TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                m_robotDrive.getRightFeedforward(), m_robotDrive.getKinematics(), 11.0);

        TrajectoryConfig forwardConfig = new TrajectoryConfig(0.4 * Drive.kMaxSpeed, Drive.kMaxAcceleration)
                .setKinematics(m_robotDrive.getKinematics()).addConstraint(voltageConstraint);

        // All units in meters except the ones in radians (I think)
        // starts facing positive x by default not positive y

        // new Translation2d(x, y)
        m_trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0.00, 0.00, new Rotation2d(Math.PI*2)),

        List.of(new Translation2d(1.37, 0.15),
        new Translation2d(1.52, 0.76),
        new Translation2d(1.80, 1.37),
        new Translation2d(3.81, 1.52),
        new Translation2d(5.94, 1.37),
        new Translation2d(6.10, 0.76),
        new Translation2d(6.25, 0.15),
        new Translation2d(6.86, 0.00),
        new Translation2d(7.62, 0.76),
        new Translation2d(6.86, 1.52),
        new Translation2d(6.35, 1.37),
        new Translation2d(6.20, 0.76),
        new Translation2d(6.04, 0.30),
        new Translation2d(3.91, 0.15),
        new Translation2d(2.23, 0.30),
        new Translation2d(1.72, 0.90),
        new Translation2d(1.57, 1.50)),

        new Pose2d(0.00, 1.52, new Rotation2d(Math.PI)), forwardConfig);
    }

    public void autonomousInit() {
        super.autonomousInit();

        if (m_greg.getVoltage() >= 0 && m_greg.getVoltage() < 1) {
            m_selection = 0; //TODO
        } else if (m_greg.getVoltage() >= 1 && m_greg.getVoltage() < 2){
            m_selection = 1; //TODO
        } else if (m_greg.getVoltage() >= 2 && m_greg.getVoltage() < 3) {
            m_selection = 2; //TODO
        } else if (m_greg.getVoltage() >= 3 && m_greg.getVoltage() < 4) {
            m_selection = 3; //TODO
        } else {
            m_selection = ; //TODO
            System.out.print("Hello world");
        }
    }

    public void autonomousPeriodic() {
        switch (m_selection) {
            case ???: //TODO
                autonomous???(); //TODO
                break;
            //TODO
        }
    }

    public void autonomousPeriodicARed() {
        boolean running = false;

        if (m_state == "start") {
            changeState("first");
            System.out.println("hello");
        } else if (m_state == "first") {
            running = track(m_trajectory);
            if (!running) {
                changeState("end");
            }
        }
    }
}
