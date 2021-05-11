package frc.robot;

import frc.robot.autonomous.*;
import frc.robot.subsystems.*;

import frc.robot.utils.Debouncer;
import frc.robot.utils.MiscMath;
// This is the limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The JAVA VIM is configured to automatically run this class, and to call the
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

  private final Drive m_drive = new Drive();
  private final Hood m_hood = new Hood();
  private final Shooter m_shooter = new Shooter();

  private final Hopper m_hopper = new Hopper();
  private final Intake m_intake = new Intake();

  private final DigitalInput m_cellDetector = new DigitalInput(8);
  private final Debouncer m_cellDetectorDebouncer = new Debouncer();

  private final XboxController m_controller = new XboxController(1);
  private final Joystick m_stick = new Joystick(0);

  private AutonomousBase m_auto;
  private final SendableChooser<AutonomousBase> m_autoChooser = new SendableChooser<AutonomousBase>();

  private final Climbing m_winch = new Climbing();

  public Robot() {
    m_autoChooser.setDefaultOption("No_Auto", new NoAutonomous(m_drive));

    m_auto = m_autoChooser.getSelected();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putData("Auto_Choice", m_autoChooser);

    m_winch.setDefaultCommand(
        new RunCommand(() -> m_winch.setWinch(-kMaxWinchSpeed * m_controller.getY(Hand.kRight)), m_winch));

    m_drive.setDefaultCommand(new RunCommand(() -> m_drive.drive(kMaxJoySpeed * MiscMath.deadband(-m_stick.getY()),
        kMaxJoyTurn * MiscMath.deadband(-m_stick.getX()))));

    m_hood.setDefaultCommand(new RunCommand(() -> m_hood.move(kMaxHoodSpeed * m_controller.getY(Hand.kLeft))));

    new JoystickButton(m_controller, Button.kA.value).whenPressed(() -> {
      m_hopper.setHopper(-0.6);
      m_hopper.setKicker(0.24);
    }).whenReleased(() -> {
      m_hopper.setHopper(0);
      m_hopper.setKicker(0);
    });

    new JoystickButton(m_controller, Button.kB.value).whenPressed(() -> {
      m_hopper.setHopper(0.6);
      m_hopper.setKicker(-0.24);
    }).whenReleased(() -> {
      m_hopper.setHopper(0);
      m_hopper.setKicker(0);
    });

    new JoystickButton(m_controller, Button.kY.value).whenPressed(() -> {
      m_intake.setIntake(-0.3);
    }).whenReleased(() -> {
      m_intake.setIntake(0);
    });

    new JoystickButton(m_stick, 11).whenPressed(new InstantCommand(m_intake::raise, m_intake));
    new JoystickButton(m_stick, 10).whenPressed(new InstantCommand(m_intake::lower, m_intake));

    
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
    // SmartDashboard.putNumber("pose_x",
    // m_robotDrive.getPose().getTranslation().getX());
    // SmartDashboard.putNumber("pose_y",
    // m_robotDrive.getPose().getTranslation().getY());
    // SmartDashboard.putNumber("pose_rot",
    // m_robotDrive.getPose().getRotation().getDegrees());
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
    m_auto = m_autoChooser.getSelected();

    // m_auto.autonomousInit();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // m_auto.autonomousPeriodic();
  }

  /**
   * This function is called when entering operator control.
   */
  @Override
  public void teleopInit() {
    // m_auto.autonomousEnd();
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
      double heading_error = tx.getDouble(0.0);
      double angle_error = ty.getDouble(0.0);

      if (heading_error > 1.0) {
        steerCommand = kTargetP * heading_error + kMinTargetCommand;
      } else if (heading_error < -1.0) {
        steerCommand = kTargetP * heading_error - kMinTargetCommand;
      }

      // m_aimer.aim(angle_error);
    } else {
      driveCommand = kMaxJoySpeed * MiscMath.deadband(-m_stick.getY());
      steerCommand = kMaxJoyTurn * MiscMath.deadband(-m_stick.getX());

    }

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

  }

  @Override
  public void testInit() {
    // m_auto.autonomousEnd();
  }

  @Override
  public void testPeriodic() {

  }
}
