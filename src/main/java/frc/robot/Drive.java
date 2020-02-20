package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

    private static final double kTrackWidth = 0.619; // meters
    private static final double kWheelRadius = 0.076; // meters
    private static final int kEncoderResolution = 1024;

    private static final boolean kGyroReversed = false;

    private final SpeedController m_leftMaster = new Spark(2);
    private final SpeedController m_leftFollower = new Spark(3);
    private final SpeedController m_rightMaster = new Spark(0);
    private final SpeedController m_rightFollower = new Spark(1);

    private final Encoder m_leftEncoder = new Encoder(4, 5);
    private final Encoder m_rightEncoder = new Encoder(6, 7);

    private final Gyro m_gyro = new ADXRS450_Gyro();

    private final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftMaster, m_leftFollower);
    private final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightMaster, m_rightFollower);

    private final PIDController m_leftPIDController = new PIDController(16.6, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(16.6, 0, 0);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(2.24, 2.28, 0.506);

    private double m_maxOutput = 10.0;

    public Drive() {
        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

        resetEncoders();

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        m_leftGroup.setInverted(true);
        m_rightGroup.setInverted(true);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance());
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
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
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

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        speeds.leftMetersPerSecond = Util.clamp(speeds.leftMetersPerSecond, -m_maxOutput, m_maxOutput);
        speeds.rightMetersPerSecond = Util.clamp(speeds.rightMetersPerSecond, -m_maxOutput, m_maxOutput);

        double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
        double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(),
                speeds.rightMetersPerSecond);

        //double leftVoltage = leftOutput + leftFeedforward;
        //double rightVoltage = rightOutput + rightFeedforward;
        double leftVoltage = leftFeedforward;
        double rightVoltage = rightFeedforward;
        m_leftGroup.setVoltage(leftVoltage);
        m_rightGroup.setVoltage(rightVoltage);
    }

    public void setSpeeds(double left , double right) {
        setSpeeds(new DifferentialDriveWheelSpeeds(left,right));
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot    Angular velocity in rad/s.
     */
    public void drive(double xSpeed, double rot) {
        double wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, -rot));
        setSpeeds(wheelSpeeds);
    }
}
