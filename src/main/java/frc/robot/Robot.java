package frc.robot;

import frc.robot.autonomous.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.utils.MiscMath;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Button;

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
  private final Vision m_vision = new Vision();
  private final Climbing m_climbing = new Climbing();

  private final Wheel m_wheel = new Wheel();
  private final Hopper m_hopper = new Hopper();
  private final Intake m_intake = new Intake();


  private final XboxController m_controller = new XboxController(1);
  private final Joystick m_stick = new Joystick(0);

  private CommandBase m_auto;
  private final SendableChooser<CommandBase> m_autoChooser = new SendableChooser<CommandBase>();

  public Robot() {
    m_autoChooser.setDefaultOption("No_Auto", new NoAutonomous());
    m_autoChooser.addOption("PAS", new PickupAndShoot(m_drive, m_intake, m_hopper, m_vision, m_hood, m_shooter));
    m_autoChooser.addOption("OFS", new OurTrenchFishtailShoot(m_drive, m_intake, m_hopper, m_vision, m_hood, m_shooter));
    m_autoChooser.addOption("BIAS", new JustBackItUpAndShoot(m_drive, m_intake, m_hopper, m_vision, m_hood, m_shooter));
    m_auto = m_autoChooser.getSelected();
    
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putData("Auto_Choice", m_autoChooser);

    m_climbing.setDefaultCommand(new RunCommand(
        () -> m_climbing.setWinch(-kMaxWinchSpeed * m_controller.getY(XboxController.Hand.kRight)), m_climbing));

    m_drive.setDefaultCommand(new RunCommand(() -> m_drive.drive(kMaxJoySpeed * MiscMath.deadband(-m_stick.getY()),
        kMaxJoyTurn * MiscMath.deadband(-m_stick.getX())), m_drive));

    m_hood.setDefaultCommand(
        new RunCommand(() -> m_hood.move(kMaxHoodSpeed * m_controller.getY(XboxController.Hand.kLeft)), m_hood));

    new JoystickButton(m_controller, XboxController.Button.kA.value).whenPressed(new RunCommand(() -> {
      m_hopper.setHopper(-0.6);
      m_hopper.setKicker(0.24);
    }, m_hopper)).whenReleased(new RunCommand(() -> {
      m_hopper.setHopper(0);
      m_hopper.setKicker(0);
    }, m_hopper));

    new JoystickButton(m_controller, XboxController.Button.kB.value).whenPressed(new RunCommand(() -> {
      m_hopper.setHopper(0.6);
      m_hopper.setKicker(-0.24);
    }, m_hopper)).whenReleased(new RunCommand(() -> {
      m_hopper.setHopper(0);
      m_hopper.setKicker(0);
    }, m_hopper));

    new JoystickButton(m_controller, XboxController.Button.kY.value).whenPressed(new RunCommand(() -> {
      m_intake.setIntake(-0.3);
    }, m_intake)).whenReleased(new RunCommand(() -> {
      m_intake.setIntake(0);
    }, m_intake));

    new JoystickButton(m_controller, XboxController.Button.kStart.value).whenPressed(new RunCommand(() -> {
      m_wheel.setWheel(0.6);
    }, m_wheel)).whenReleased(new RunCommand(() -> {
      m_wheel.setWheel(0);
    }, m_wheel));

    new JoystickButton(m_stick, 11).whenPressed(new InstantCommand(m_intake::raise, m_intake));
    new JoystickButton(m_stick, 10).whenPressed(new InstantCommand(m_intake::lower, m_intake));
    new JoystickButton(m_stick, 9).whenHeld(new AimCommand(m_vision, m_hood, m_drive));

    new JoystickButton(m_controller, XboxController.Button.kX.value)
        .whenPressed(new InstantCommand(m_wheel::raiseOrLower, m_wheel));

    new JoystickButton(m_stick, 6).whenPressed(new InstantCommand(m_climbing::engage, m_climbing));
    new JoystickButton(m_stick, 7).whenPressed(new InstantCommand(m_climbing::disEngage, m_climbing));
    
    new JoystickButton(m_controller, XboxController.Button.kBumperLeft.value).whenHeld(
        new ShootCommand(m_shooter, m_hopper, () -> m_controller.getTriggerAxis(XboxController.Hand.kRight) > 0.2));

    new Button(() -> m_controller.getTriggerAxis(XboxController.Hand.kLeft) > 0.2)
        .whenHeld(new IntakeCommand(m_intake, m_hopper, false));

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
    CommandScheduler.getInstance().run();
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

    if (m_auto != null) {
      m_auto.schedule();
    }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called when entering operator control.
   */
  @Override
  public void teleopInit() {

    if (m_auto != null) {
      m_auto.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }
}
