package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class Autonomous {
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;

        private static final double kShootTimeStart = 1.5;
        private static final double kShootTimeEnd = 5.7;

        protected final Timer m_timer = new Timer();

        protected final Drive m_robotDrive;
        protected final Hood m_aimer;
        protected final Shooter m_shooter;
        protected final SpeedController m_kicker;
        protected final SpeedController m_hopper;
        protected final SpeedController m_intake;
        protected final DoubleSolenoid m_intakeLift;

        protected final RamseteController m_follower;

        protected DifferentialDriveWheelSpeeds m_prevSpeeds;
        protected double m_prevTime;

        protected String m_state;

        public Autonomous(Drive robotDrive, Hood aimer, Shooter shooter, SpeedController kicker, SpeedController hopper,
                        SpeedController intake, DoubleSolenoid intakeLift) {
                m_robotDrive = robotDrive;
                m_aimer = aimer;
                m_shooter = shooter;
                m_kicker = kicker;
                m_hopper = hopper;
                m_intake = intake;
                m_intakeLift = intakeLift;

                m_follower = new RamseteController(kRamseteB, kRamseteZeta);
        }

        public void autonomousInit() {
                m_prevTime = 0;

                m_prevSpeeds = new DifferentialDriveWheelSpeeds(0, 0);

                changeState("start");

                m_robotDrive.resetPID();
                m_robotDrive.resetOdometry(new Pose2d());
        }

        protected boolean track(Trajectory trajectory) {
                double curTime = m_timer.get();
                double dt = curTime - m_prevTime;

                DifferentialDriveWheelSpeeds targetWheelSpeeds = m_robotDrive.getKinematics().toWheelSpeeds(
                                m_follower.calculate(m_robotDrive.getPose(), trajectory.sample(curTime)));

                double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
                double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

                double leftOutput;
                double rightOutput;

                double leftFeedforward = m_robotDrive.getLeftFeedforward().calculate(leftSpeedSetpoint,
                                (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

                double rightFeedforward = m_robotDrive.getRightFeedforward().calculate(rightSpeedSetpoint,
                                (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

                leftOutput = leftFeedforward + m_robotDrive.getLeftPIDController()
                                .calculate(m_robotDrive.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);

                rightOutput = rightFeedforward + m_robotDrive.getRightPIDController()
                                .calculate(m_robotDrive.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

                m_robotDrive.setVoltages(leftOutput, rightOutput);

                m_prevTime = curTime;
                m_prevSpeeds = targetWheelSpeeds;

                return m_prevTime <= trajectory.getTotalTimeSeconds();
        }

        protected boolean target() {
                NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
                NetworkTableEntry tx = table.getEntry("tx");
                NetworkTableEntry ty = table.getEntry("ty");
                NetworkTableEntry tv = table.getEntry("tv");

                boolean steerReady = false;
                boolean aimReady = false;

                if (tv.getBoolean(false)) {
                        double driveCommand = 0.0;
                        double steerCommand = 0.0;

                        double heading_error = -tx.getDouble(0.0);
                        double angle_error = ty.getDouble(0.0);

                        if (heading_error > 1.0) {
                                steerCommand = Robot.kTargetP * heading_error + Robot.kMinTargetCommand;
                        } else if (heading_error < -1.0) {
                                steerCommand = Robot.kTargetP * heading_error - Robot.kMinTargetCommand;
                        } else if (m_timer.get() > 1.5) {
                                steerReady = true;
                        }

                        aimReady = m_aimer.aim(angle_error);

                        m_robotDrive.drive(driveCommand, steerCommand);
                } else {
                        m_aimer.move(0.0);
                        m_robotDrive.drive(0.0, 0.0);
                }

                return !(steerReady && aimReady);
        }

        protected boolean shoot() {
                m_shooter.shoot(Shooter.kShootSpeed);

                if (m_timer.get() > kShootTimeStart && m_shooter.getRate() > Shooter.kShootReadySpeed) {
                        m_kicker.set(-1);
                        m_hopper.set(0.65);
                } else {
                        m_kicker.set(0);
                        m_hopper.set(0);
                }

                return m_timer.get() <= kShootTimeEnd;
        }

        protected boolean dropIntake() {
                if (m_intakeLift != null) {
                        if (m_intakeLift.get() != Value.kForward) {
                                m_intakeLift.set(Value.kForward);
                                return true;
                        } else {
                                m_intakeLift.set(Value.kOff);
                                return false;
                        }
                } else {
                        return false;
                }
        }

        protected void changeState(String state) {
                m_state = state;

                m_timer.reset();
                m_timer.start();

                m_robotDrive.setVoltages(0, 0);
                m_intake.set(0.0);
                m_shooter.setVoltage(0.0);
                m_kicker.set(0.0);
                m_hopper.set(0.0);
        }

        public void autonomousPeriodic() {
        }

        public void autonomousEnd() {
                m_timer.stop();
                m_robotDrive.setVoltages(0, 0);
        }
}