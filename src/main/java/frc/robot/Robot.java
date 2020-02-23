/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// This is the limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final double kMaxJoySpeed = 3.0; // meters per sec
  private static final double kMaxJoyTurn = 5.0; // radians per sec
  private static final double kMaxHoodSpeed = 0.5; // ratio
  private static final double kMaxWinchSpeed = 1.0; // ratio

  private final DoubleSolenoid m_climberLift;
  private final DoubleSolenoid m_intakeLift;
  private final DoubleSolenoid m_controlPanelLift;

  private final Drive m_robotDrive = new Drive();
  private final Hood m_aimer = new Hood();
  private final Autonomous m_auto = new Autonomous(m_robotDrive);

  private final Spark m_kicker = new Spark(4);
  private final Spark m_shooter = new Spark(5);
  private final Spark m_hopper = new Spark(RobotConstants.getInstance().kHopper);
  private final Spark m_intake = new Spark(7);
  private final Spark m_controlPanel;
  private final Spark m_winch;

  private final XboxController m_controller = new XboxController(1);
  private final Joystick m_stick = new Joystick(0);

  private final double m_KpAim = -0.085;
  private final double m_KpDistance = -0.1;
  private final double m_min_aim_command = -.5;

  public Robot() {
    if (!RobotConstants.kPractice) {
      m_climberLift = new DoubleSolenoid(1, 2, 3);
      m_intakeLift = new DoubleSolenoid(1, 0, 1);
      m_controlPanelLift = new DoubleSolenoid(1, 6, 7);
      m_controlPanel = new Spark(11);
      m_winch = new Spark(10);
    }
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_auto.autonomousInit();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    m_auto.autonomousPeriodic();
  }

  /**
   * This function is called when entering operator control.
   */
  @Override
  public void teleopInit() {
    m_auto.autonomousEnd();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double driveCommand = 0.0;
    double steerCommand = 0.0;

    if (m_stick.getRawButton(9)) {
      double heading_error = -tx.getDouble(0.0);
      double distance_error = -ty.getDouble(0.0);

      if (heading_error > 1.0) {
        steerCommand = m_KpAim * heading_error + m_min_aim_command;
      } else if (heading_error < -1.0) {
        steerCommand = m_KpAim * heading_error - m_min_aim_command;
      }

      SmartDashboard.putNumber("steerCommand", steerCommand);

      // double distance_adjust = m_KpDistance * distance_error;
    } else {
      driveCommand = kMaxJoySpeed * Util.deadband(-m_stick.getY());
      steerCommand = kMaxJoyTurn * Util.deadband(m_stick.getX());
    }
    m_robotDrive.drive(driveCommand, steerCommand);

    double shooterCommand = 0;
    double intakeCommand = 0;
    double kickerCommand = 0;
    double hopperCommand = 0;

    if (m_controller.getBumper(Hand.kLeft)) {
      shooterCommand = -1;
      // TODO lower intake
      if (m_controller.getTriggerAxis(Hand.kRight) > .2) {
        kickerCommand = -1;
        hopperCommand = .65;
        // TODO: (ADD VISION TRACKER now)
      }
    } else if (m_controller.getTriggerAxis(Hand.kLeft) > .2) {
      intakeCommand = -.65;
      kickerCommand = .24;
      hopperCommand = -.45;
    }

    if (m_controller.getBButton()) {
      hopperCommand = .6;
      kickerCommand = -.2343;
    }

    if (m_controller.getYButton()) {
      intakeCommand = .5;
    }

    m_aimer.move(kMaxHoodSpeed*m_controller.getY(Hand.kLeft));

    if (m_winch != null) {
      m_winch.set(kMaxWinchSpeed*m_controller.getY(Hand.kRight));
    }

    m_shooter.set(shooterCommand);
    m_intake.set(intakeCommand);
    m_kicker.set(kickerCommand);
    m_hopper.set(hopperCommand);

    if (m_controlPanel != null) {
      if (m_controller.getXButton()) {
        m_controlPanel.set(1);
      } else {
        m_controlPanel.set(0);
      }
    }

    if (m_intakeLift != null) {
      if (m_stick.getRawButton(11)) {
        m_intakeLift.set(Value.kForward);
      } else if (m_stick.getRawButton(10)) {
        m_intakeLift.set(Value.kReverse);
      } else {
        m_intakeLift.set(Value.kOff);
      }
    }

    if (m_climberLift != null) {
      if (m_stick.getRawButton(6)) {
        m_climberLift.set(Value.kForward);
      } else if (m_stick.getRawButton(7)) {
        m_climberLift.set(Value.kReverse);
      } else {
        m_climberLift.set(Value.kOff);
      }
    }

    if (m_controlPanelLift != null) {
      if (m_stick.getRawButton(4)) {
        m_controlPanelLift.set(Value.kForward);
      } else if (m_stick.getRawButton(5)) {
        m_controlPanelLift.set(Value.kReverse);
      } else {
        m_controlPanelLift.set(Value.kOff);
      }
    }
  }

  @Override
  public void testInit() {
    m_auto.autonomousEnd();
  }

  @Override
  public void testPeriodic() {

  }
}
