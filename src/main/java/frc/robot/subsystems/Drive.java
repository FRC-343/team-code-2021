package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MiscMath;

public class Drive extends SubsystemBase {
    public static final double kMaxSpeed = 4.0; // meters per second
    public static final double kMaxAcceleration = 3.0; // meters per second squared
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

    private static final double kTrackWidth = 0.568; // meters
    private static final double kWheelRadius = 0.0762; // meters
    private static final int kEncoderResolution = 256;

    private static final boolean kGyroReversed = true;

    private final Spark m_leftMaster = new Spark(2);
    private final Spark m_leftFollower = new Spark(3);
    private final Spark m_rightMaster = new Spark(0);
    private final Spark m_rightFollower = new Spark(1);

    private final Encoder m_leftEncoder = new Encoder(2, 3);
    private final Encoder m_rightEncoder = new Encoder(0, 1);

    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftMaster, m_leftFollower);
    private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightMaster, m_rightFollower);

    private final PIDController m_leftPIDController = new PIDController(2.0, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(2.0, 0, 0);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    private final SimpleMotorFeedforward m_leftFeedforward = new SimpleMotorFeedforward(2.11, 2.86, 0.716);
    private final SimpleMotorFeedforward m_rightFeedforward = new SimpleMotorFeedforward(2.11, 2.81, 0.698);

    private final DigitalInput m_stopSensor = new DigitalInput(15);//help

    private DifferentialDriveOdometry m_odometry;

    private boolean m_PIDEnabled = false;
    private double m_maxOutput = 10.0;
    private DifferentialDriveWheelSpeeds m_wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));

    public Drive() {
        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

        resetEncoders();

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        m_leftGroup.setInverted(false);
        m_rightGroup.setInverted(true);
        m_leftEncoder.setReverseDirection(false);
        m_rightEncoder.setReverseDirection(true);
        
        SendableRegistry.setSubsystem(m_leftMaster, this.getClass().getSimpleName());
        SendableRegistry.setName(m_leftMaster, "Left Master Drive Motor Thingy");
        SendableRegistry.setSubsystem(m_rightMaster, this.getClass().getSimpleName());
        SendableRegistry.setName(m_rightMaster, "Right Master Drive Motor Thingy");
        SendableRegistry.setSubsystem(m_leftFollower, this.getClass().getSimpleName());
        SendableRegistry.setName(m_leftFollower, "Left Follower Drive Motor Thingy");
        SendableRegistry.setSubsystem(m_rightFollower, this.getClass().getSimpleName());
        SendableRegistry.setName(m_rightFollower, "Right Follower Drive Motor Thingy");

        SendableRegistry.setSubsystem(m_leftEncoder, this.getClass().getSimpleName());
        SendableRegistry.setName(m_leftEncoder, "Left Drive Encoder Thingy");
        SendableRegistry.setSubsystem(m_rightEncoder, this.getClass().getSimpleName());
        SendableRegistry.setName(m_rightEncoder, "Right Drive Encoder Thingy");

        SendableRegistry.setSubsystem(m_gyro, this.getClass().getSimpleName());
        SendableRegistry.setName(m_gyro, "Gyro Drive Thingy");

        SendableRegistry.setSubsystem(m_leftGroup, this.getClass().getSimpleName());
        SendableRegistry.setName(m_leftGroup, "Left Drive Wheel Group Thingy");
        SendableRegistry.setSubsystem(m_rightGroup, this.getClass().getSimpleName());
        SendableRegistry.setName(m_rightGroup, "Right Drive Wheel Group Thingy");

        SendableRegistry.setSubsystem(m_leftPIDController, this.getClass().getSimpleName());
        SendableRegistry.setName(m_leftPIDController, "Left Drive PID Controller Thingy");
        SendableRegistry.setSubsystem(m_rightPIDController, this.getClass().getSimpleName());
        SendableRegistry.setName(m_rightPIDController, "Right Drive PID Controller Thingy");

        SendableRegistry.setSubsystem(m_stopSensor, this.getClass().getSimpleName());
        SendableRegistry.setName(m_stopSensor, "Drive Stopping Sensor Thingy on wheel Thingy");
    }

    public PIDController getLeftPIDController() {
        return m_leftPIDController;
    }

    public PIDController getRightPIDController() {
        return m_rightPIDController;
    }

    public SimpleMotorFeedforward getLeftFeedforward() {
        return m_leftFeedforward;
    }

    public SimpleMotorFeedforward getRightFeedforward() {
        return m_rightFeedforward;
    }

    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_maxOutput = maxOutput;
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public void resetPID() {
        m_leftPIDController.reset();
        m_rightPIDController.reset();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * (kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (kGyroReversed ? -1.0 : 1.0);
    }

    public void setVoltages(double left, double right) {
        m_leftGroup.setVoltage(left);
        m_rightGroup.setVoltage(right);
        m_PIDEnabled = false;
    }

    public void drive(double xSpeed, double rot) {
        if (xSpeed > 0.0 && m_stopSensor.get()) {
            m_wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        } else {
            m_wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        }
        m_PIDEnabled = true;
    }

    @Override
    public void periodic() {
        if (m_PIDEnabled) {
            m_wheelSpeeds.leftMetersPerSecond = MiscMath.clamp(m_wheelSpeeds.leftMetersPerSecond, -m_maxOutput,
                    m_maxOutput);
            m_wheelSpeeds.rightMetersPerSecond = MiscMath.clamp(m_wheelSpeeds.rightMetersPerSecond, -m_maxOutput,
                    m_maxOutput);

            double leftFeedforward = m_leftFeedforward.calculate(m_wheelSpeeds.leftMetersPerSecond);
            double rightFeedforward = m_rightFeedforward.calculate(m_wheelSpeeds.rightMetersPerSecond);

            double leftPIDOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(),
                    m_wheelSpeeds.leftMetersPerSecond);
            double rightPIDOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(),
                    m_wheelSpeeds.rightMetersPerSecond);

            double leftOutput = leftPIDOutput + leftFeedforward;
            double rightOutput = rightPIDOutput + rightFeedforward;

            m_leftGroup.setVoltage(leftOutput);
            m_rightGroup.setVoltage(rightOutput);
        } 

        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance());
        
    }
}
