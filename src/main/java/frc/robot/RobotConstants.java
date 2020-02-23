package frc.robot;

public class RobotConstants {
    public static final boolean kPractice = false;

    // PUT CONSTANTS HERE WITH A BOGUS DEFAULT VALUE AND THEN PUT IN ROBOT-SPECIFIC FILES
    public int kHoodMotor = -1;
    public int kHopper = -1;
    public double kDriveLeftS = 0.0;
    public double kDriveLeftV = 0.0;
    public double kDriveLeftA = 0.0;
    public double kDriveRightS = 0.0;
    public double kDriveRightV = 0.0;
    public double kDriveRightA = 0.0;


    private static RobotConstants m_instance;

    public static RobotConstants getInstance() {
        if (m_instance == null) {
            if (kPractice) {
                m_instance = new PracticeRobotConstants();
            }
            else {
                m_instance = new CompRobotConstants();
            }
        }

        return m_instance;
    }
}
