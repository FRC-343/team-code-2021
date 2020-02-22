package frc.robot;

public class RobotConstants {
    public static final boolean kPractice = false;

    // PUT CONSTANTS HERE WITH A BOGUS DEFAULT VALUE AND THEN PUT IN ROBOT-SPECIFIC FILES
    public int kHoodMotor = -1;
    public int kHopper = -1;
    public double kDriveS = 0.0;
    public double kDriveV = 0.0;
    public double kDriveA = 0.0;


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
