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

    private final Trajectory m_firstARed;
    private final Trajectory m_secondARed;
    private final Trajectory m_thirdARed;
    private final Trajectory m_firstABlue;
    private final Trajectory m_firstBRed;
    private final Trajectory m_firstBBlue;

    private enum Selection {
        NONE, A_RED, A_BLUE, B_RED, B_BLUE
    };

    private Selection m_selection;

    public AutonomousGalactic(Drive robotDrive, Hood aimer, Shooter shooter, SpeedController kicker,
            SpeedController hopper, SpeedController intake, DoubleSolenoid intakeLift, AnalogInput greg) {
        super(robotDrive, aimer, shooter, kicker, hopper, intake, intakeLift);

        m_greg = greg;

        m_selection = Selection.NONE;

        TrajectoryConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
                m_robotDrive.getRightFeedforward(), m_robotDrive.getKinematics(), 11.0);

        TrajectoryConfig forwardConfig = new TrajectoryConfig(0.22 * Drive.kMaxSpeed, Drive.kMaxAcceleration)
                .setKinematics(m_robotDrive.getKinematics()).addConstraint(voltageConstraint);

        // All units in meters except the ones in radians (I think)
        // starts facing positive x by default not positive y

        // new Translation2d(x, y)
        m_firstARed = TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0)), List.of(),
                new Pose2d(1.52, 0.0, new Rotation2d(0)), forwardConfig);

        m_secondARed = TrajectoryGenerator.generateTrajectory(new Pose2d(1.52, 0.0, new Rotation2d(0)),
                List.of(new Translation2d(1.91, -0.76), new Translation2d(3.05, -0.76)),
                new Pose2d(4.8, 1.02, new Rotation2d(0)), forwardConfig);

        m_thirdARed = TrajectoryGenerator.generateTrajectory(new Pose2d(4.8, 1.02, new Rotation2d(0)), List.of(),
                new Pose2d(7.6, 1.02, new Rotation2d(0)), forwardConfig);

        m_firstABlue = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, new Rotation2d(0)), List.of(new Translation2d(3.05, -1.3),
                        new Translation2d(3.96, -1), new Translation2d(5.05, 0.76), new Translation2d(6.0, 0.00)),
                new Pose2d(7.6, 0.0, new Rotation2d(0)), forwardConfig);
        m_firstBRed = TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0)),
                List.of(new Translation2d(2.0, 0.76), new Translation2d(3.25, -0.76), new Translation2d(5.45, 1.14)),
                new Pose2d(7.6, 1.14, new Rotation2d(0)), forwardConfig);

        m_firstBBlue = TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0)),
                List.of(new Translation2d(3.81, -0.76), new Translation2d(5.33, 0.76), new Translation2d(6.86, -0.76)),
                new Pose2d(7.6, -1.52, new Rotation2d(0)), forwardConfig);
    }

    public void autonomousInit() {
        super.autonomousInit();
        m_selection = Selection.B_BLUE;

    }

    public void autonomousPeriodic() {
        switch (m_selection) {
            case A_RED:
                autonomousPeriodicARed();
                break;
            case A_BLUE:
                autonomousPeriodicABlue();
                break;
            case B_RED:
                autonomousPeriodicBRed();
                break;
            case B_BLUE:
                autonomousPeriodicBBlue();
                break;
            default:
                break;

        }
    }

    /*
     * /////A Red Activate /////
     * 
     * intake Drive forward 2 units and rotate Math.PI/2 to the right (clockwise,
     * facing neg y) Trajectory curve (final 2 power cells): start at c3, end at a6
     * travel through d5 (waypoint) (rotated Math.PI/2 left, conterclockwise, facing
     * pos x), travel to a11 (5 units foward)
     * 
     */
    public void autonomousPeriodicARed() {
        boolean running = false;

        if (m_state == "start") {
            changeState("intake_drop");
        } else if (m_state == "intake_drop") {
            running = dropIntake();
            if (!running) {
                changeState("first");
            }
        } else if (m_state == "first") {
            m_intake.set(0.65);
            running = track(m_firstARed);
            if (!running) {
                changeState("second");
            }
        } else if (m_state == "second") {
            m_intake.set(0.65);
            running = track(m_secondARed);
            if (!running) {
                changeState("third");
            }
        } else if (m_state == "third") {
            running = track(m_thirdARed);

            if (!running) {
                changeState("end");
                for (int i = 0; i < 100; i++) {
                    System.out.println("Touchdown");
                }
            }
        }
    } //////// B RED //////////
    /*
     * Intake run all time
     * 
     * 
     * 
     * 
     * 
     * 
     * 
     */

    public void autonomousPeriodicBRed() {
        boolean running = false;

        if (m_state == "start") {
            changeState("intake_drop");
        } else if (m_state == "intake_drop") {
            running = dropIntake();
            if (!running) {
                changeState("first");
            }
        } else if (m_state == "first") {
            m_intake.set(0.65);
            running = track(m_firstBRed);
            if (!running) {
                changeState("end");
            }

        }
    }

    public void autonomousPeriodicABlue() {
        boolean running = false;

        if (m_state == "start") {
            changeState("intake_drop");
        } else if (m_state == "intake_drop") {
            running = dropIntake();
            if (!running) {
                changeState("first");
            }
        } else if (m_state == "first") {
            m_intake.set(0.65);
            running = track(m_firstABlue);
            if (!running) {
                changeState("end");
            }

        }
    }

    public void autonomousPeriodicBBlue() {
        boolean running = false;

        if (m_state == "start") {
            changeState("intake_drop");
        } else if (m_state == "intake_drop") {
            running = dropIntake();
            if (!running) {
                changeState("first");
            }
        } else if (m_state == "first") {
            m_intake.set(0.65);
            running = track(m_firstBBlue);
            if (!running) {
                changeState("end");
            }

        }
    }
}