/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.autonomous.*;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

// This is the limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final double kMaxJoySpeed = 3.0; // meters per sec
  public static final double kMaxJoyTurn = 5.0; // radians per sec
  public static final double kMaxHoodSpeed = 0.5; // ratio
  public static final double kMaxWinchSpeed = 1.0; // ratio

  public static final double kTargetP = -0.055;
  public static final double kMinTargetCommand = -0.35;

  private static final Color kRed = new Color(0.518311, 0.344971, 0.136963);
  private static final Color kGreen = new Color(0.1689, 0.575439, 0.25585);
  private static final Color kBlue = new Color(0.1267, 0.4160, 0.4575);
  private static final Color kYellow = new Color(0.320068, 0.558105, 0.122070);

  private final DoubleSolenoid m_climberLift;
  private final DoubleSolenoid m_intakeLift;
  // private final DoubleSolenoid m_controlPanelLift;

  private final Drive m_robotDrive = new Drive();
  private final Hood m_aimer = new Hood();
  private final Shooter m_shooter = new Shooter();

  private final Spark m_kicker = new Spark(4);
  private final Spark m_hopper = new Spark(RobotConstants.getInstance().kHopper);
  private final Spark m_intake = new Spark(7);
  private final Spark m_controlPanel;
  private final Spark m_winch;

  private final Autonomous m_autoNone;
  private final Autonomous m_autoET;
  private final Autonomous m_autoSimple;
  private final Autonomous m_autoOT;
  private final Autonomous m_autoCone;

  private final ColorSensorV3 m_color;
  private final DigitalInput m_cellDetector;
  private final Debouncer m_cellDetectorDebouncer = new Debouncer();

  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final XboxController m_controller = new XboxController(1);
  private final Joystick m_stick = new Joystick(0);

  private enum Auto {
    NO_AUTONOMOUS, AUTONOMOUS_SIMPLE, AUTONOMOUS_ET, AUTONOMOUS_OT,AUTONOMOUS_CONE
  };

  private Autonomous m_auto;
  private final SendableChooser<Auto> m_autoChooser = new SendableChooser<Auto>();

  public Robot() {
    m_intake.setInverted(true);

    if (!RobotConstants.kPractice) {
      m_climberLift = new DoubleSolenoid(1, 2, 3);
      m_intakeLift = new DoubleSolenoid(1, 0, 1);
      // m_controlPanelLift = new DoubleSolenoid(1, 6, 7);
      m_controlPanel = new Spark(11);
      m_controlPanel.setInverted(true);
      m_winch = new Spark(10);
      m_color = new ColorSensorV3(Port.kOnboard);
      m_cellDetector = new DigitalInput(8);
    }

    m_autoNone = new Autonomous(m_robotDrive, m_aimer, m_shooter, m_kicker, m_hopper, m_intake, m_intakeLift);
    m_autoET = new AutonomousET(m_robotDrive, m_aimer, m_shooter, m_kicker, m_hopper, m_intake, m_intakeLift);
    m_autoSimple = new AutonomousSimple(m_robotDrive, m_aimer, m_shooter, m_kicker, m_hopper, m_intake, m_intakeLift);
    m_autoOT = new AutonomousOT(m_robotDrive, m_aimer, m_shooter, m_kicker, m_hopper, m_intake, m_intakeLift);
    m_autoCone = new AutonomousCone(m_robotDrive, m_aimer, m_shooter, m_kicker, m_hopper, m_intake, m_intakeLift);
    m_auto = m_autoNone;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_colorMatcher.addColorMatch(kRed);
    m_colorMatcher.addColorMatch(kGreen);
    m_colorMatcher.addColorMatch(kBlue);
    m_colorMatcher.addColorMatch(kYellow);

    m_autoChooser.setDefaultOption("Auto_Simple", Auto.AUTONOMOUS_SIMPLE);
    m_autoChooser.addOption("Auto_ET", Auto.AUTONOMOUS_ET);
    m_autoChooser.addOption("Auto_OT", Auto.AUTONOMOUS_OT);
    m_autoChooser.addOption("Auto_Cone", Auto.AUTONOMOUS_CONE);
    m_autoChooser.addOption("No_Auto", Auto.NO_AUTONOMOUS);
    SmartDashboard.putData("Auto_Choice", m_autoChooser);
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
    SmartDashboard.putNumber("pose_x", m_robotDrive.getPose().getTranslation().getX());
    SmartDashboard.putNumber("pose_y", m_robotDrive.getPose().getTranslation().getY());
    SmartDashboard.putNumber("pose_rot", m_robotDrive.getPose().getRotation().getDegrees());

    ColorMatchResult detectedColor = m_colorMatcher.matchClosestColor(m_color.getColor());

    if (detectedColor.color == kRed) {
      SmartDashboard.putString("color_detected", "red");
    } else if (detectedColor.color == kGreen) {
      SmartDashboard.putString("color_detected", "green");
    } else if (detectedColor.color == kBlue) {
      SmartDashboard.putString("color_detected", "blue");
    } else if (detectedColor.color == kYellow) {
      SmartDashboard.putString("color_detected", "yellow");
    }
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
    if (m_autoChooser.getSelected() == Auto.AUTONOMOUS_ET) {
      m_auto = m_autoET;
    } else if (m_autoChooser.getSelected() == Auto.AUTONOMOUS_SIMPLE) {
      m_auto = m_autoSimple;
    } else if (m_autoChooser.getSelected() == Auto.AUTONOMOUS_OT) {
      m_auto = m_autoOT;
    }  else if (m_autoChooser.getSelected() == Auto.AUTONOMOUS_CONE) {
      m_auto = m_autoCone;
    } else {
      m_auto = m_autoNone;
    }

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

    double driveCommand = 0.0;
    double steerCommand = 0.0;

    if (m_stick.getRawButton(9)) {
      double heading_error = -tx.getDouble(0.0);
      double angle_error = ty.getDouble(0.0);

      if (heading_error > 1.0) {
        steerCommand = kTargetP * heading_error + kMinTargetCommand;
      } else if (heading_error < -1.0) {
        steerCommand = kTargetP * heading_error - kMinTargetCommand;
      }

      m_aimer.aim(angle_error);
    } else {
      driveCommand = kMaxJoySpeed * Util.deadband(-m_stick.getY());
      steerCommand = kMaxJoyTurn * Util.deadband(m_stick.getX());

      m_aimer.move(kMaxHoodSpeed * m_controller.getY(Hand.kLeft));
    }
    m_robotDrive.drive(driveCommand, steerCommand);

    double shooterCommand = 0;
    double intakeCommand = 0;
    double kickerCommand = 0;
    double hopperCommand = 0;

    if (m_controller.getBumper(Hand.kLeft)) {
      shooterCommand = Shooter.kShootSpeed;
      if (m_controller.getTriggerAxis(Hand.kRight) > 0.2) {
        kickerCommand = -1;
        if (m_shooter.getRate() > Shooter.kShootReadySpeed) {
          hopperCommand = 0.65;
        }
      }
    } else if (m_controller.getTriggerAxis(Hand.kLeft) > 0.2) {
      intakeCommand = 0.65;
      if (m_cellDetector == null || (m_cellDetectorDebouncer.isReady(m_cellDetector.get()) && !m_cellDetector.get())) {
        hopperCommand = -0.6;
        kickerCommand = 0.24;
      }
    }

    if (m_controller.getAButton()) {
      hopperCommand = -0.6;
      kickerCommand = 0.24;
    } else if (m_controller.getBButton()) {
      hopperCommand = 0.6;
      kickerCommand = -0.24;
    }

    if (m_controller.getYButton()) {
      intakeCommand = -0.299999999999;
    }

    if (m_winch != null) {
      double witchCommand = -kMaxWinchSpeed * m_controller.getY(Hand.kRight);
      if (witchCommand > 0.0 || m_controller.getBackButton()) {
        m_winch.set(witchCommand);
      } else {
        m_winch.set(0.0);
      }
    }

    m_shooter.shoot(shooterCommand);
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
      if ((DriverStation.getInstance().getMatchTime() < 30.0 || m_stick.getRawButtonPressed(2))
          && m_stick.getRawButton(6)) {
        m_climberLift.set(Value.kForward);
      } else if (m_stick.getRawButton(7)) {
        m_climberLift.set(Value.kReverse);
      } else {
        m_climberLift.set(Value.kOff);
      }
    }

    /*if (m_controlPanelLift != null) {
      if (m_controller.getXButtonReleased()) {
        m_controlPanelLift.set(Value.kForward);
      } else if (m_controller.getXButtonPressed()) {
        m_controlPanelLift.set(Value.kReverse);
      } else {
        m_controlPanelLift.set(Value.kOff);
      }
    }*/
  } 

  @Override
  public void testInit() {
    m_auto.autonomousEnd();
  }

  @Override
  public void testPeriodic() {

  }
}
